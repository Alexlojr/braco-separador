// Firmware do braço robótico — Arduino Nano + módulo PCA9685 (I2C).
//
// Sketch único: faz a operação normal (comandos vindos da Raspberry Pi) E os
// testes de bancada da montagem. Ter dois sketches separados significava
// trocar de firmware no meio da calibração e lembrar de trocar o baudrate
// junto — quando não se lembra, a serial devolve lixo e parece defeito de
// hardware. Aqui não há o que trocar.
//
// ---------------------------------------------------------------------------
// LIGAÇÃO
// ---------------------------------------------------------------------------
//   PCA9685 VCC -> 5V do Nano (lógica)  |  PCA9685 V+ -> fonte externa 5V/10A
//   PCA9685 GND -> GND do Nano, comum com o GND da fonte externa
//   PCA9685 SDA -> A4 do Nano           |  PCA9685 SCL -> A5 do Nano
//   Servos: base=canal 0, ombro=canal 1, cotovelo=canal 2, garra=canal 3
//
// Biblioteca: "Adafruit PWM Servo Driver Library" (Gerenciador de Bibliotecas)
//
// ---------------------------------------------------------------------------
// PROTOCOLO — texto, uma linha por comando, 115200 baud
// ---------------------------------------------------------------------------
// Operação (o lado Python só usa estes):
//   SET <id> <ang>        move a junta para o ângulo, com movimento suavizado
//   GET <id> | GET ALL    posição atual (durante o movimento, a intermediária)
//   HOME                  volta todas para a pose de repouso
//   BUSY                  "BUSY 1" enquanto alguma junta se move, senão "BUSY 0"
//   STOP                  para tudo onde está, agora
//
// Bancada / calibração (passo 2 do roteiro):
//   DEMO [<graus_s>]      roda a sequência de pega e solta sozinho, sem PC
//   SWEEP <id> <de> <ate> vai e volta na janela, devagar, até um STOP
//   LIM                   mostra os limites de todas as juntas
//   LIM <id> <min> <max>  redefine o limite de uma junta
//   SPEED <graus_s>       velocidade média do BRAÇO (padrão 160 °/s)
//   SPEEDG [<graus_s>]    velocidade só da garra. 0 = SEM SUAVIZAÇÃO, que é
//                         o padrão; acima de ~500 não muda nada
//   DETACH <id>|ALL       corta o PWM: dá pra mexer a junta com a mão
//   ATTACH <id>|ALL       religa o PWM na posição atual
//   RAW <id> <pulso>      escreve a contagem de pulso crua (102-492 a 50 Hz)
//   PULSO <id> <min> <max>  guarda a calibração de pulso achada com RAW
//   HELP                  lista os comandos
//
// Sensor de corrente (ACS712 em série com o V+ da garra, saída em A0):
//   AMP                   "AMP <corrente_A> <adc_medio> <adc_zero>"
//   ZERO                  mede o zero do sensor agora (garra parada!)
//   SENS <mV_por_A>       sensibilidade: 185 (5 A), 100 (20 A), 66 (30 A)
//   LIMAMP [<amperes>]    limite de travamento; 0 desliga; sem arg mostra
//
// Mensagem não solicitada:
//   STALL <canal> <corrente_A>     a garra travou; o firmware está aliviando
//   OK ALIVIO <canal> <angulo>     alívio concluído, parou neste ângulo
//
// Erros respondem "ERR <motivo> <linha>". Todo comando responde alguma coisa:
// silêncio significa que a linha não chegou.
//
// ---------------------------------------------------------------------------
// SEGURANÇA DA CALIBRAÇÃO
// ---------------------------------------------------------------------------
// Use SWEEP em janelas de ~20°, nunca de ponta a ponta. Se a junta bater num
// limite mecânico no meio do curso, o servo trava e continua puxando corrente
// — é assim que se descasca engrenagem de MG90S. Com STOP a um comando de
// distância, dá pra cortar antes de estragar.

#include <stdlib.h>          // atoi(), atof() — ver a nota sobre sscanf abaixo
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t NUM_SERVOS = 4;
const uint8_t CANAL_PCA[NUM_SERVOS] = {0, 1, 2, 3};

// A garra é a única junta que o resto do código precisa saber QUAL é: ela
// não participa da cinemática e não pode dividir lote com o braço.
// Diferente de canal_medido, que diz onde o sensor de corrente está ligado
// e é configurável, este é a montagem física e não muda em tempo de execução.
const uint8_t IDX_GARRA = 3;
const uint8_t IDX_OMBRO = 1;
const uint8_t IDX_COTOVELO = 2;

// ---------------------------------------------------------------------------
// LIMITE ACOPLADO COTOVELO x OMBRO
// ---------------------------------------------------------------------------
// O limite do cotovelo depende de onde o ombro esta: com o braco recolhido
// o antebraco encosta nele antes de chegar no lim_min.
//
// Medido com DETACH e a mao, procurando o menor s2 que ainda nao encosta:
//
//     s1=100 -> encosta em s2=135      ajuste
//     s1=130 -> encosta em s2= 90      ajuste
//     s1=115 -> encosta em s2=115      VALIDACAO: a reta previa 112,5
//     s1=160 -> s2=70 livre, com folga
//     s1= 90 -> encosta em s2=136, mas ali quem bate e a GARRA
//
// A margem de 15 cobre a ultima linha: outra colisao, outra fronteira, e a
// reta e otimista nessa ponta.
//
// Vale nos DOIS sentidos - recolher o ombro com o cotovelo ja baixo chega
// na mesma colisao pelo outro caminho.
const float ACOP_A = 285.0;
const float ACOP_B = 1.5;
const float ACOP_MARGEM = 15.0;

const int PWM_FREQ = 50;
const unsigned long INTERVALO_PASSO_MS = 20;   // 50 Hz de atualização

// Calibração de pulso, em contagens de 0-4095.
//
// NÃO use o par 150-600 dos exemplos da Adafruit: aquilo é para 60 Hz. A
// 50 Hz cada contagem vale 4,883 us, e 150-600 viram 732-2930 us — além dos
// 500-2400 us que SG90 e MG90S aceitam, empurrando o servo contra o batente
// na parte de cima do curso. O equivalente correto é 102-492.
//
// Ponto de partida; cada peça varia. Ache com RAW partindo de 300, grave
// com PULSO.
int pulso_min[NUM_SERVOS] = {102, 102, 102, 102};
int pulso_max[NUM_SERVOS] = {492, 492, 492, 492};

// Limites por junta, em graus de servo. MEDIDOS NA BANCADA, E SÃO A FONTE:
// config/braco.json e pegada_manual.py copiam daqui.
//
// O coletar.py empurra os limites do JSON por LIM ao conectar, então um JSON
// desatualizado AFROUXA na conexão um limite necessário. Por isso
// testa_limites_sincronizados() falha se os três divergirem.
//
// Abaixo de 70 o cotovelo gira para dentro e encosta na estrutura. A garra
// é invertida: 100 abre, 140 fecha. Curso de 40°, achado em 08/11/2026
// depois de ajustar a montagem — antes eram 17° (120 a 137).
//
// O 100 é FIM DE CURSO, não posição de trabalho: ali as pinças ficam retas
// e o quadrilátero da garra para no ponto de inflexão. A operação abre em
// 105 (90 mm de abertura), que é o garra_aberta_graus do braco.json.
float lim_min[NUM_SERVOS] = {  0.0, 90.0,  70.0,  100.0};
float lim_max[NUM_SERVOS] = {180.0, 180.0, 160.0, 140.0};

// Pose de repouso do HOME.
//
// No boot o firmware assume esta pose e escreve o pulso de uma vez, sem
// suavizar — é o movimento mais brusco do sistema, e acontece a cada
// energização. Ligue com o braço já perto dela.
//
// Ombro 120 e cotovelo 120 deixam a garra a 112 mm do eixo e 59 mm de
// altura. A garra em 105 fica 5° dentro do fim de curso, porque encostar
// nele toda vez que liga não é boa ideia.
//
// A base em 50 — e não 90, apontada para a área de coleta — tira o braço
// do caminho da CÂMERA. Ao repouso a garra fica 18 mm antes da borda da
// placa e a 59 mm de altura, e numa vista oblíqua isso se projeta em cima
// do canto próximo dela: o objeto que estiver ali não é detectado. Em 50 a
// garra vai para (72, 86), 22 mm fora da borda +X da placa. O depósito
// fica do outro lado (base 129), sem conflito.
//
// Sobre o raio: dobrar mais o braço quase NÃO o encurta. De 140/120 a
// 115/160 o raio fica entre 138 e 148 mm; o que muda é a altura, de 50
// para 102 mm. 120/120 é a exceção que recua de verdade, para 112 mm, e é
// por isso que é esta a pose.
//
// 120/120 fica exatamente no limite acoplado calculado (s2 >= 285 - 1,5·s1
// + 15 dá 120 para s1=120). Os 15° são a margem do modelo, então sobra
// essa margem e não mais: não recue o ombro além de 120 sem medir.
float pose_repouso[NUM_SERVOS] = {50.0, 120.0, 120.0, 135.0};

float atual[NUM_SERVOS];      // posição sendo escrita agora (segue a curva)
float alvo[NUM_SERVOS];       // para onde o movimento está indo
float origem[NUM_SERVOS];     // de onde partiu — a curva interpola origem->alvo
bool  ligado[NUM_SERVOS];     // false = PWM cortado, junta solta

unsigned long t_inicio[NUM_SERVOS];   // quando este movimento começou
unsigned long duracao[NUM_SERVOS];    // quanto ele deve durar, em ms

float velocidade = 90.0;      // graus por segundo, MÉDIA (ver suaviza())

// Velocidade só da garra, com SPEEDG. Ela não está na cadeia cinemática, não
// carrega massa e não precisa seguir caminho nenhum: não há motivo para andar
// no ritmo do braço.
//
// 0 = SEM SUAVIZAÇÃO, e é o padrão.
//
// O defeito que isso corrige foi diagnosticado quando o curso era de 17°,
// ou 37 contagens do PCA9685: suavizado, o início do movimento — onde a
// smootherstep tem derivada zero — ficava abaixo de uma contagem, o pulso
// escrito não mudava, e a garra tremia sem fechar. O curso hoje é de 40°
// (87 contagens) e a margem é maior, mas o padrão continua 0: a garra não
// carrega massa nem segue caminho, e o impacto nas duas pontas, inaceitável
// para o braço, aqui não tem consequência.
float velocidade_garra = 0.0;
const float VEL_GARRA_MAX = 1000.0;

// Escrever o pulso de uma vez NÃO significa que o servo chegou: ele leva o
// tempo mecânico dele, e o BUSY precisa dizer 1 nesse intervalo. Senão o lado
// Python manda a pose seguinte com a garra a meio curso.
//
// 300 °/s era folha de dados, e estava MUITO otimista. No primeiro ciclo real,
// em 09/11/2026, a garra foi fechando durante a subida do braço e só terminou
// no fim do curso dele — ou seja, os 29° do fechamento levaram perto de um
// segundo, e não os 97 ms que 300 °/s previam.
//
// O motivo provável não é o servo: é a tensão. A garra fecha com os outros
// três servos segurando o braço estendido, e a queda nos jumpers tira dela
// justamente o torque de que precisa. Com DETACH nos outros, deve subir.
//
// 60 °/s é conservador de propósito. Errar para LENTO custa meio segundo de
// ciclo; errar para rápido faz o braço subir com a peça a meio aperto, que
// derruba a peça. Para ajustar: cronometre SET 3 105 -> SET 3 134 com o braço
// estendido e energizado, e divida os 29° pelo tempo.
const float VEL_MECANICA_GARRA = 60.0;    // graus/s, para estimar o BUSY
const unsigned long GARRA_ASSENTA_MIN_MS = 60;
unsigned long t_garra_fim = 0;            // ate quando o BUSY conta a garra

// IMPULSO: movimento curto de garra passa primeiro pelo fim de curso aberto.
//
// Medido na bancada em 09/11/2026: mandar a garra de 140 para 130 ela demora
// muito ou simplesmente nao vai. Mandar 100 antes, e so depois 130, funciona
// sempre. O motivo e atrito estatico — o curso util inteiro tem 40 graus, e
// um comando de 10 graus vira uma variacao de pulso pequena demais para
// vencer o atrito de partida da engrenagem com o paralelogramo da garra.
// Piora com o braco estendido, quando a queda de tensao nos jumpers tira
// torque justamente de quem precisa de um empurrao.
//
// Correr ate o batente primeiro resolve porque o movimento longo parte com
// margem de sobra, e o segundo trecho sai de um ponto conhecido e com o
// mecanismo ja solto.
//
// Abaixo deste curso, aplica o impulso. Acima, vai direto.
const float GARRA_IMPULSO_MIN = 15.0;
float garra_pendente = -1.0;    // alvo real depois do impulso; <0 = nenhum
bool garra_impulsionou = false; // ultimo defineAlvo usou impulso?
bool acoplamento_apertou = false;         // ultimo defineAlvo bateu no acoplamento?

// Um movimento curto demais fica com duração de poucos milissegundos e volta
// a ser degrau. Este piso garante que até um ajuste de 1° seja suavizado.
const unsigned long DURACAO_MIN_MS = 80;

// Movimentos disparados dentro desta janela contam como um lote só e recebem
// todos a MESMA duração. A sequência de coleta manda "SET 0 / SET 1 / SET 2"
// em linhas seguidas, com microssegundos entre elas: sem isso, cada junta
// chegaria num instante diferente e a garra faria um caminho torto no ar,
// diferente do que a cinemática planejou.
const unsigned long JANELA_LOTE_MS = 50;
unsigned long t_lote = 0;
unsigned long duracao_lote = 0;
bool no_lote[NUM_SERVOS];

// Estado do SWEEP. Fica no laco principal em vez de num for() bloqueante:
// bloquear significaria não conseguir ler o STOP enquanto a junta se move,
// que é justamente quando ele é necessário.
bool    sweep_ativo = false;
uint8_t sweep_id = 0;
float   sweep_de = 0, sweep_ate = 0;

// ---------------------------------------------------------------------------
// Sequência de demonstração
// ---------------------------------------------------------------------------
// Pega e solta rodando no próprio firmware, sem PC — para vídeo e para a
// apresentação. Mesmo roteiro do pegada_manual.py.
//
// Ângulos de SERVO: não passa por offset nem por cinemática, então funciona
// mesmo sem calibração. -1 = esta junta não se mexe neste passo.
//
// Ao editar: recolha antes de girar a base, que é onde o ombro está no maior
// torque. Ajustar pelo pegada_manual.py --passo é mais rápido, porque não
// precisa recompilar a cada número.
const int16_t demo_passos[][NUM_SERVOS] PROGMEM = {
  // base  ombro  cotovelo  garra
  {   -1,   130,     140,     -1 },   //  1 recolhe
  {   -1,    -1,      -1,    105 },   //  2 abre a garra
  {   90,    -1,      -1,     -1 },   //  3 gira a base para a peça
  {   -1,   160,     120,     -1 },   //  4 estende por cima da peça
  {   -1,   170,     110,     -1 },   //  5 desce até a peça
  {   -1,    -1,      -1,    134 },   //  6 FECHA a garra
  {   -1,   150,     130,     -1 },   //  7 levanta com a peça
  {   -1,   130,     140,     -1 },   //  8 recolhe para girar
  {   40,    -1,      -1,     -1 },   //  9 gira a base para o depósito
  {   -1,   160,     120,     -1 },   // 10 estende sobre o depósito
  {   -1,    -1,      -1,    105 },   // 11 ABRE a garra, solta a peça
  {   -1,   130,     140,     -1 },   // 12 recolhe
  {   50,   120,     120,     -1 },   // 13 volta ao repouso
};
const uint8_t DEMO_N = sizeof(demo_passos) / sizeof(demo_passos[0]);

// Pausa depois de cada passo chegar. O servo para no ângulo, mas o acrílico
// ainda oscila; importa principalmente antes de fechar a garra.
const unsigned long DEMO_PAUSA_MS = 400;

bool          demo_ativo = false;
bool          demo_assentando = false;   // chegou, contando a pausa
uint8_t       demo_passo = 0;
unsigned long t_demo = 0;

unsigned long t_ultimo_passo = 0;

// ---------------------------------------------------------------------------
// Sensor de corrente ACS712
// ---------------------------------------------------------------------------
// Instalado em série com o fio vermelho (V+) do servomotor da garra, então vê
// só a corrente dela. Alimentado pelo 5 V do Nano e lido em A0.
//
// Alimentar pelo 5 V do Nano não é detalhe: o ACS712 é ratiométrico e o ADC
// usa VCC como referência, então no mesmo trilho a alimentação se cancela na
// conta. Alimentado pela fonte de potência, a leitura carregaria justamente
// a flutuação que o sensor existe para medir.
//
//   corrente_A = (adc_medio - adc_zero) / (mV_por_A * 1023 / 5000)
//
// O zero não é 511,5 — cada peça tem offset. Medido no boot, refaz com ZERO.

const uint8_t PINO_ACS = A0;      // A4 e A5 são o I2C do PCA9685
uint8_t canal_medido = 3;         // garra

float acs_mv_por_amp   = 185.0;   // versão de 5 A; 100 = 20 A, 66 = 30 A
float acs_zero_adc     = 511.5;   // substituído pela medição no boot
float limite_stall_amp = 0.0;     // 0 = alívio desligado (padrão até calibrar)

// Servomotor puxa corrente em picos sincronizados com o PWM de 50 Hz. Uma
// leitura crua pula muito; a média de uma janela que cobre vários períodos
// do PWM é o que se usa. 32 amostras a 200 Hz = 160 ms = 8 períodos.
const unsigned long INTERVALO_AMOSTRA_MS = 5;
const uint8_t N_AMOSTRAS = 32;
int  amostras[N_AMOSTRAS];
uint8_t idx_amostra = 0;
long soma_amostras = 0;
bool janela_cheia = false;
unsigned long t_ultima_amostra = 0;

// Alívio de travamento. Quando a garra fecha sobre algo que não cede, o
// servomotor não chega ao alvo e fica solicitado no limite: é isso que aquece
// e descasca engrenagem. O firmware não tem realimentação de posição, então
// não sabe ONDE a garra parou; sabe só que a corrente subiu. A resposta é
// recuar o ângulo comandado, um grau por vez, no sentido de onde veio, até a
// corrente cair. O ponto onde ela cai é o contato sem excesso de força — que,
// para uma peça retida por encaixe, é exatamente a pega desejada.
const unsigned long GRACA_STALL_MS     = 300;   // ignora o pico de partida
const unsigned long INTERVALO_ALIVIO_MS = 50;
const float PASSO_ALIVIO_GRAUS  = 1.0;
const float ALIVIO_MAX_GRAUS    = 15.0;   // desiste depois disto: não é travamento
bool  em_alivio = false;
float alivio_acumulado = 0.0;
float direcao_alivio = 0.0;               // +1 ou -1
unsigned long t_ultimo_alivio = 0;

// Trava depois de desistir. Sem ela, se a corrente continua alta após o
// recuo máximo, o laço seguinte reentra em alívio, recua mais 15°, desiste,
// reentra... até a garra chegar ao limite da junta e soltar a peça, enquanto
// inunda a serial. Só destrava quando a corrente cai abaixo do limite ao
// menos uma vez, ou quando chega comando novo para a garra.
bool stall_desistiu = false;

char linha[48];
uint8_t n_linha = 0;

// ---------------------------------------------------------------------------
// Saída para os servos
// ---------------------------------------------------------------------------

// Converte ângulo em contagem de pulso SEM truncar para inteiro no caminho.
// O firmware antigo fazia map((long)angulo, ...), o que jogava fora a casa
// decimal que a cinemática inversa calcula. Com ~160 mm de alcance, 1° vale
// cerca de 2,8 mm na ponta da garra — arredondar custava até 1,4 mm.
void escrevePulso(uint8_t id, float angulo) {
  float fracao = angulo / 180.0;
  long pulso = lround(pulso_min[id] + fracao * (pulso_max[id] - pulso_min[id]));
  pulso = constrain(pulso, 0, 4095);
  pwm.setPWM(CANAL_PCA[id], 0, (uint16_t)pulso);
}

void desligaSaida(uint8_t id) {
  pwm.setPWM(CANAL_PCA[id], 0, 4096);   // bit de "full off" do PCA9685
}

// Menor variação de ângulo que o PCA9685 consegue expressar neste canal.
// Com a faixa 102-492 são 390 contagens para 180 graus: 0,46 grau por
// contagem, ou cerca de 2 mm na ponta da garra com o braço esticado.
//
// Esse é o piso de resolução de todo o sistema. Mandar incrementos menores
// que isso não produz movimento nenhum: o valor escrito é o mesmo, e o
// servomotor ainda tem a banda morta dele por cima.
float grausPorContagem(uint8_t id) {
  return 180.0 / (float)(pulso_max[id] - pulso_min[id]);
}

float limitaAoIntervalo(uint8_t id, float angulo) {
  return constrain(angulo, lim_min[id], lim_max[id]);
}

// Menor cotovelo seguro para um dado ombro, e o inverso. Nunca afrouxam o
// limite de caixa: so apertam.
float cotoveloMinimoPara(float s1) {
  float m = ACOP_A - ACOP_B * s1 + ACOP_MARGEM;
  return m > lim_min[IDX_COTOVELO] ? m : lim_min[IDX_COTOVELO];
}

float ombroMinimoPara(float s2) {
  float m = (ACOP_A + ACOP_MARGEM - s2) / ACOP_B;
  return m > lim_min[IDX_OMBRO] ? m : lim_min[IDX_OMBRO];
}

// Aplica o acoplamento. A junta PARCEIRA e avaliada na posicao mais
// restritiva que ela assume durante o trajeto — o menor valor entre onde ela
// esta e para onde esta indo. Usar so o alvo deixaria passar uma pose final
// valida cujo caminho atravessa a zona de colisao.
float limitaAcoplado(uint8_t id, float angulo, bool *apertou) {
  float minimo;
  if (id == IDX_COTOVELO) {
    float s1 = atual[IDX_OMBRO] < alvo[IDX_OMBRO] ? atual[IDX_OMBRO] : alvo[IDX_OMBRO];
    minimo = cotoveloMinimoPara(s1);
  } else if (id == IDX_OMBRO) {
    float s2 = atual[IDX_COTOVELO] < alvo[IDX_COTOVELO] ? atual[IDX_COTOVELO] : alvo[IDX_COTOVELO];
    minimo = ombroMinimoPara(s2);
  } else {
    return angulo;
  }
  if (angulo < minimo) { *apertou = true; return minimo; }
  return angulo;
}

// ---------------------------------------------------------------------------
// Suavização do movimento
// ---------------------------------------------------------------------------
// Interpolação linear resolve o meio do movimento e não as pontas: a
// velocidade salta de zero para o valor cheio e volta a zero de repente, e
// aceleração instantânea é o tranco.
//
// suaviza() é a smootherstep, 6p^5 - 15p^4 + 10p^3. Primeira E segunda
// derivadas nulas em p=0 e p=1, então nem velocidade nem aceleração dão
// salto, inclusive no retorno e na inversão do sweep.
//
// Custo: o pico fica ~1,9x a média. SPEED é a MÉDIA — "SPEED 60" quer dizer
// que 60° levam 1 segundo, não que o servo nunca passa de 60 °/s.

float suaviza(float p) {
  if (p <= 0.0) return 0.0;
  if (p >= 1.0) return 1.0;
  return p * p * p * (p * (p * 6.0 - 15.0) + 10.0);
}

void defineAlvo(uint8_t id, float angulo) {
  angulo = limitaAoIntervalo(id, angulo);
  acoplamento_apertou = false;
  angulo = limitaAcoplado(id, angulo, &acoplamento_apertou);
  unsigned long agora = millis();

  // Um comando novo para a garra encerra qualquer alívio em curso e destrava
  // uma desistência anterior: quem mandou sabe o que quer, e a janela de
  // graça do movimento novo cobre o pico de partida.
  if (id == canal_medido) {
    em_alivio = false;
    stall_desistiu = false;
  }

  // A GARRA NÃO ENTRA NO LOTE, e tem velocidade própria.
  //
  // O lote existe para as juntas da cadeia cinemática chegarem juntas: sem
  // isso a ponta descreve no ar um caminho diferente do que a cinemática
  // planejou. A garra não está na cadeia — não há o que sincronizar.
  //
  // Deixá-la no lote era um defeito com sintoma enganoso. Ao colar várias
  // linhas no monitor serial, tudo cai no mesmo lote e a garra herda a
  // duração do movimento mais longo: o fechamento esticado para os 4 s de um
  // giro de ombro cai abaixo da zona morta do servo — com os 17° de curso da
  // época davam 10 °/s. A garra treme, não fecha, e parece que o comando não
  // chegou. Com os 35° de hoje a margem é maior, mas o defeito é o mesmo.
  //
  // O pegada_manual.py e a tabela DEMO contornavam isso por convenção, cada
  // um mandando a garra sozinha na sua etapa. Agora o firmware garante.
  if (id == IDX_GARRA) {
    // Curso curto: desvia para o batente aberto e guarda o alvo real para
    // quando o impulso terminar. Ver GARRA_IMPULSO_MIN.
    //
    // Nao se aplica quando o proprio alvo ja esta perto do batente: ali o
    // segundo trecho seria curto de novo, e o servo esta encostado no fim de
    // curso, que e a condicao em que ele menos precisa de ajuda.
    garra_impulsionou = false;
    float curso_pedido = fabs(angulo - atual[id]);
    bool perto_do_batente = fabs(angulo - lim_min[id]) < GARRA_IMPULSO_MIN;

    if (curso_pedido > 0.05 && curso_pedido < GARRA_IMPULSO_MIN
        && !perto_do_batente && garra_pendente < 0.0) {
      garra_pendente = angulo;
      angulo = lim_min[id];
      garra_impulsionou = true;
    }

    origem[id] = atual[id];
    alvo[id] = angulo;
    no_lote[id] = false;
    t_inicio[id] = agora;
    float curso = fabs(angulo - origem[id]);

    if (velocidade_garra <= 0.0) {
      // Sem suavizacao: duracao 0 faz atualizaMovimento() escrever o alvo
      // direto, no proximo passo.
      duracao[id] = 0;
      unsigned long mecanico = (unsigned long)(curso / VEL_MECANICA_GARRA * 1000.0);
      if (mecanico < GARRA_ASSENTA_MIN_MS) mecanico = GARRA_ASSENTA_MIN_MS;
      t_garra_fim = agora + mecanico;
    } else {
      unsigned long natural = (unsigned long)(curso / velocidade_garra * 1000.0);
      duracao[id] = (natural < DURACAO_MIN_MS) ? DURACAO_MIN_MS : natural;
      t_garra_fim = agora + duracao[id];
    }
    return;
  }

  // Abre um lote novo se o anterior já esfriou. Movimentos disparados juntos
  // compartilham a duração para chegarem juntos.
  if (agora - t_lote > JANELA_LOTE_MS) {
    t_lote = agora;
    duracao_lote = 0;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) no_lote[i] = false;
  }

  // Partir da posição ATUAL, não do alvo anterior: se um SET novo chega no
  // meio de um movimento, continuar de onde o servo realmente está é o que
  // evita o salto de posição — que seria outro tranco.
  origem[id] = atual[id];
  alvo[id] = angulo;
  no_lote[id] = true;

  unsigned long natural = (unsigned long)(fabs(angulo - origem[id]) / velocidade * 1000.0);
  if (natural < DURACAO_MIN_MS) natural = DURACAO_MIN_MS;
  if (natural > duracao_lote) duracao_lote = natural;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (no_lote[i]) {
      t_inicio[i] = t_lote;
      duracao[i] = duracao_lote;
    }
  }
}

// Um impulso pendente ainda e movimento: dizer BUSY 0 entre as duas pernas
// faria o lado Python mandar a pose seguinte com a garra no batente, e nao
// no angulo que ele pediu.
bool algumaEmMovimento() {
  if (garra_pendente >= 0.0) return true;
  return algumaEmMovimentoInterno();
}


bool algumaEmMovimentoInterno() {
  unsigned long agora = millis();
  // A garra sem suavizacao tem duracao 0, entao o laco abaixo nao a enxerga.
  // Ela ainda esta andando: o pulso foi escrito, o servo e que nao teletransporta.
  if (ligado[IDX_GARRA] && (long)(t_garra_fim - agora) > 0) return true;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (ligado[i] && agora - t_inicio[i] < duracao[i]) return true;
  }
  return false;
}

void paraTudo() {
  sweep_ativo = false;
  demo_ativo = false;
  em_alivio = false;
  t_garra_fim = 0;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    // Congela onde está: origem = alvo = posição atual, duração zerada.
    origem[i] = atual[i];
    alvo[i] = atual[i];
    duracao[i] = 0;
  }
}

// ---------------------------------------------------------------------------
// Sensor de corrente
// ---------------------------------------------------------------------------

float contagensPorAmp() {
  // mV/A -> contagens de ADC por ampère, para VCC de 5 V e ADC de 10 bits.
  // Ratiométrico: a razão vale para qualquer VCC, desde que sensor e ADC
  // compartilhem o mesmo trilho (ver comentário na declaração das variáveis).
  return acs_mv_por_amp * 1023.0 / 5000.0;
}

void amostraCorrente() {
  unsigned long agora = millis();
  if (agora - t_ultima_amostra < INTERVALO_AMOSTRA_MS) return;
  t_ultima_amostra = agora;

  int leitura = analogRead(PINO_ACS);
  soma_amostras -= amostras[idx_amostra];
  amostras[idx_amostra] = leitura;
  soma_amostras += leitura;
  idx_amostra++;
  if (idx_amostra >= N_AMOSTRAS) {
    idx_amostra = 0;
    janela_cheia = true;
  }
}

float adcMedio() {
  uint8_t n = janela_cheia ? N_AMOSTRAS : idx_amostra;
  if (n == 0) return acs_zero_adc;
  return (float)soma_amostras / n;
}

// Corrente com sinal. Negativa significa que IP+ e IP- estão invertidos no
// borne — inofensivo, e o alívio usa o módulo. Vale trocar os fios só para o
// número ficar bonito no relatório.
float correnteAmp() {
  return (adcMedio() - acs_zero_adc) / contagensPorAmp();
}

// Mede o zero com tudo parado. Bloqueia ~500 ms de propósito: é comando de
// bancada, e a média longa é o que dá um zero estável.
void medeZero() {
  long soma = 0;
  const int N = 100;
  for (int i = 0; i < N; i++) {
    soma += analogRead(PINO_ACS);
    delay(5);
  }
  acs_zero_adc = (float)soma / N;

  // Recomeça a janela móvel, senão ela fica contaminada com leituras feitas
  // contra o zero antigo.
  soma_amostras = 0;
  idx_amostra = 0;
  janela_cheia = false;
  for (uint8_t i = 0; i < N_AMOSTRAS; i++) amostras[i] = 0;
}

void verificaStall() {
  if (limite_stall_amp <= 0.0) return;      // desligado
  if (!janela_cheia) return;                 // ainda sem média confiável
  uint8_t g = canal_medido;
  if (!ligado[g]) return;                    // garra solta: não há o que aliviar

  float corrente = fabs(correnteAmp());
  unsigned long agora = millis();

  if (em_alivio) {
    // Histerese: só declara aliviado bem abaixo do limite, para não ficar
    // oscilando em torno dele.
    if (corrente < limite_stall_amp * 0.8) {
      em_alivio = false;
      Serial.print(F("OK ALIVIO "));
      Serial.print(g);
      Serial.print(' ');
      Serial.println(atual[g], 1);
      return;
    }
    if (alivio_acumulado >= ALIVIO_MAX_GRAUS) {
      // Recuou o máximo e a corrente não caiu: não é travamento mecânico
      // (pode ser curto, sensor mal calibrado, ou limite baixo demais).
      // Desiste de aliviar e avisa, em vez de recuar sem fim. A trava
      // impede reentrar enquanto a corrente não cair.
      em_alivio = false;
      stall_desistiu = true;
      Serial.print(F("ERR STALL_PERSISTE "));
      Serial.print(g);
      Serial.print(' ');
      Serial.println(corrente, 2);
      return;
    }
    if (agora - t_ultimo_alivio >= INTERVALO_ALIVIO_MS) {
      t_ultimo_alivio = agora;
      float novo = limitaAoIntervalo(g, atual[g] + direcao_alivio * PASSO_ALIVIO_GRAUS);
      atual[g] = novo;
      alvo[g] = novo;
      origem[g] = novo;
      alivio_acumulado += PASSO_ALIVIO_GRAUS;
      escrevePulso(g, novo);
    }
    return;
  }

  if (corrente <= limite_stall_amp) {
    stall_desistiu = false;           // corrente normalizou: pode voltar a vigiar
    return;
  }

  if (stall_desistiu) return;         // já recuou o máximo e não adiantou

  // A corrente de partida de um movimento normal também passa do limite por
  // um instante. Só conta como travamento depois da janela de graça.
  if (agora - t_inicio[g] < GRACA_STALL_MS) return;

  // Travou. Congela a garra onde o comando está e começa a recuar no sentido
  // de onde ela veio. Se não estava se movendo (travou parada, segurando),
  // recua para o meio do curso, que é o palpite neutro sem realimentação.
  em_alivio = true;
  alivio_acumulado = 0.0;
  duracao[g] = 0;
  alvo[g] = atual[g];

  float de_onde_veio = origem[g] - atual[g];
  if (fabs(de_onde_veio) > 0.5) {
    direcao_alivio = (de_onde_veio > 0) ? 1.0 : -1.0;
  } else {
    float meio = (lim_min[g] + lim_max[g]) / 2.0;
    direcao_alivio = (meio > atual[g]) ? 1.0 : -1.0;
  }
  origem[g] = atual[g];
  t_ultimo_alivio = agora;

  Serial.print(F("STALL "));
  Serial.print(g);
  Serial.print(' ');
  Serial.println(corrente, 2);
}

// Segunda perna do impulso: o batente ja foi alcancado, agora vai ao alvo.
//
// Fica no loop e nao dentro do defineAlvo porque o firmware nao bloqueia: o
// primeiro trecho precisa de tempo real para acontecer, e esperar dentro do
// defineAlvo travaria a serial e o resto do braco junto.
void concluiImpulsoGarra() {
  if (garra_pendente < 0.0) return;
  // Subtracao e nao comparacao direta, como no resto do arquivo: millis()
  // volta a zero depois de ~49 dias, e "millis() < t_garra_fim" trava o
  // impulso para sempre se a virada cair no meio dele.
  if ((long)(t_garra_fim - millis()) > 0) return;

  float destino = garra_pendente;
  garra_pendente = -1.0;    // zera ANTES de chamar, senao o defineAlvo ve um
                            // impulso em curso e recusa o proprio alvo
  defineAlvo(IDX_GARRA, destino);
  garra_impulsionou = false;   // ja foi reportado no SET que originou
}


void atualizaMovimento() {
  unsigned long agora = millis();
  if (agora - t_ultimo_passo < INTERVALO_PASSO_MS) return;
  t_ultimo_passo = agora;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!ligado[i]) continue;

    float posicao;
    if (duracao[i] == 0) {
      posicao = alvo[i];
    } else {
      unsigned long decorrido = agora - t_inicio[i];
      if (decorrido >= duracao[i]) {
        posicao = alvo[i];
        duracao[i] = 0;          // movimento encerrado
      } else {
        float p = (float)decorrido / (float)duracao[i];
        posicao = origem[i] + (alvo[i] - origem[i]) * suaviza(p);

        // Corta a cauda da curva.
        //
        // A smootherstep chega ao alvo com velocidade zero, então os últimos
        // graus caem abaixo de meia contagem de pulso e viram ticks isolados
        // de 0,46° cada vez mais espaçados — o braço "acertando aos poucos".
        // Grudar no alvo chega no mesmo destino sem a cauda, porque o que
        // falta é menor que a resolução do hardware.
        //
        // NÃO zera duracao[i]: cada junta atinge a meia contagem num instante
        // diferente, e zerar aqui desmonta a sincronia do lote. O movimento
        // segue "ocupado" até o fim da duração combinada, só não se mexe
        // mais. Reaplicar a cada passo é seguro porque a curva é monótona.
        if (fabs(alvo[i] - posicao) < grausPorContagem(i) * 0.5) {
          posicao = alvo[i];
        }
      }
    }

    // Não reescreve o pulso se nada mudou: com o braço parado isso zera o
    // tráfego I2C em vez de repetir o mesmo valor 50 vezes por segundo.
    if (fabs(posicao - atual[i]) < 0.01) continue;

    atual[i] = posicao;
    escrevePulso(i, atual[i]);
  }

  // O sweep só inverte quando a junta chegou de fato. Como cada trecho é uma
  // curva suavizada inteira, a inversão acontece com o servo já parado — que
  // é justamente onde o sweep antigo dava o tranco mais feio.
  if (sweep_ativo && duracao[sweep_id] == 0) {
    defineAlvo(sweep_id, (fabs(alvo[sweep_id] - sweep_de) < 0.05) ? sweep_ate : sweep_de);
  }
}

// Confere a tabela ANTES de mover: um passo que mistura a garra com outra
// junta não é erro visível, é a garra abrindo devagar demais para vencer a
// zona morta. Melhor recusar em terra.
bool demoTabelaValida() {
  for (uint8_t k = 0; k < DEMO_N; k++) {
    uint8_t quantas = 0;
    bool tem_garra = false;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      int16_t a = (int16_t)pgm_read_word(&demo_passos[k][i]);
      if (a < 0) continue;
      quantas++;
      if (i == IDX_GARRA) tem_garra = true;
    }
    if (quantas == 0 || (tem_garra && quantas > 1)) {
      Serial.print(F("ERR DEMO_PASSO "));
      Serial.println(k + 1);
      return false;
    }
  }
  return true;
}

// Avança a sequência. Mesmo desenho do SWEEP: estado no laço principal, nada
// de delay(), para que STOP continue sendo lido enquanto o braço se move.
void atualizaDemo() {
  if (!demo_ativo) return;

  if (algumaEmMovimento()) {
    demo_assentando = false;
    return;
  }

  // Chegou: começa a contar a pausa de assentamento.
  if (!demo_assentando) {
    demo_assentando = true;
    t_demo = millis();
    return;
  }
  if (millis() - t_demo < DEMO_PAUSA_MS) return;

  if (demo_passo >= DEMO_N) {
    demo_ativo = false;
    Serial.println(F("OK DEMO FIM"));
    return;
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    int16_t a = (int16_t)pgm_read_word(&demo_passos[demo_passo][i]);
    if (a >= 0) defineAlvo(i, (float)a);
  }

  Serial.print(F("DEMO "));
  Serial.print(demo_passo + 1);
  Serial.print('/');
  Serial.println(DEMO_N);

  demo_passo++;
  demo_assentando = false;
}

// ---------------------------------------------------------------------------
// Comandos
// ---------------------------------------------------------------------------

bool idValido(int id) {
  return id >= 0 && id < NUM_SERVOS;
}

void respondePos(uint8_t id) {
  Serial.print(F("POS "));
  Serial.print(id);
  Serial.print(' ');
  Serial.println(atual[id], 1);
}

void respondeLimites() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(F("LIM "));
    Serial.print(i);
    Serial.print(' ');
    Serial.print(lim_min[i], 1);
    Serial.print(' ');
    Serial.println(lim_max[i], 1);
  }
}

void mostraAjuda() {
  Serial.println(F("SET <id> <ang> | GET <id>|ALL | HOME | BUSY | STOP"));
  Serial.println(F("DEMO [<g/s>]  sequencia de pega e solta, sozinho"));
  Serial.println(F("SWEEP <id> <de> <ate> | LIM [<id> <min> <max>] | SPEED <g/s>"));
  Serial.println(F("SPEEDG [<g/s>]  velocidade so da garra, fora do lote"));
  Serial.println(F("DETACH <id>|ALL | ATTACH <id>|ALL | RAW <id> <pulso>"));
  Serial.println(F("PULSO <id> <min> <max> | HELP"));
  Serial.println(F("AMP | ZERO | SENS <mV/A> | LIMAMP [<A>]"));
}

void erroArgs(const char *linha) {
  Serial.print(F("ERR BAD_ARGS "));
  Serial.println(linha);
}


// Divide a linha em tokens separados por espaço ou tabulação, terminando cada
// um com '\0' no próprio buffer. Devolve quantos tokens saíram.
uint8_t divideEmTokens(char *texto, char *tok[], uint8_t maximo) {
  uint8_t n = 0;
  char *p = texto;

  while (*p && n < maximo) {
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') break;
    tok[n++] = p;
    while (*p && *p != ' ' && *p != '\t') p++;
    if (*p) *p++ = '\0';
  }
  return n;
}

// ---------------------------------------------------------------------------
// Interpretação dos comandos
// ---------------------------------------------------------------------------
// atoi() e atof(), NUNCA sscanf(): o avr-libc linka um scanf sem ponto
// flutuante. sscanf("SET %d %f") compila sem aviso, roda, devolve 1 em vez
// de 2, e todo comando com decimal responde ERR UNKNOWN como se a linha
// estivesse errada. Mesma razão de sprintf("%f") não funcionar aqui.

void processaComando(char *texto) {
  for (char *p = texto; *p; p++) *p = toupper(*p);

  char *tok[4];
  uint8_t n = divideEmTokens(texto, tok, 4);
  if (n == 0) return;

  const char *cmd = tok[0];

  // ---- operação ---------------------------------------------------------

  if (strcmp(cmd, "SET") == 0 && n == 3) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    sweep_ativo = false;              // um SET manual cancela o sweep em curso
    defineAlvo(id, atof(tok[2]));
    Serial.print(F("OK SET "));
    Serial.print(id);
    Serial.print(' ');
    Serial.print(alvo[id], 1);
    // Diz QUAL limite mordeu. Um angulo saturado em silencio e o braco indo
    // para um lugar que ninguem pediu, e a diferenca entre o limite de caixa
    // e o acoplado muda o que se faz a respeito.
    if (acoplamento_apertou) Serial.print(F(" ACOPLADO"));
    // O impulso MOVE a garra para um lugar que nao foi pedido, ainda que por
    // instantes. Calar isso seria esconder movimento de quem esta operando.
    if (garra_impulsionou) {
      Serial.print(F(" IMPULSO->"));
      Serial.print(garra_pendente, 1);
    }
    Serial.println();

  } else if (strcmp(cmd, "GET") == 0 && n == 2 && strcmp(tok[1], "ALL") == 0) {
    Serial.print(F("POS ALL"));
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      Serial.print(' ');
      Serial.print(i);
      Serial.print(':');
      Serial.print(atual[i], 1);
    }
    Serial.println();

  } else if (strcmp(cmd, "GET") == 0 && n == 2) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    respondePos(id);

  } else if (strcmp(cmd, "HOME") == 0 && n == 1) {
    sweep_ativo = false;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) defineAlvo(i, pose_repouso[i]);
    Serial.println(F("OK HOME"));

  } else if (strcmp(cmd, "STOP") == 0 && n == 1) {
    paraTudo();
    Serial.println(F("OK STOP"));

  } else if (strcmp(cmd, "BUSY") == 0 && n == 1) {
    Serial.print(F("BUSY "));
    Serial.println(algumaEmMovimento() ? 1 : 0);

  // ---- bancada ----------------------------------------------------------

  } else if (strcmp(cmd, "DEMO") == 0 && (n == 1 || n == 2)) {
    if (!demoTabelaValida()) return;
    if (n == 2) {
      float v = atof(tok[1]);
      if (v > 0) velocidade = v;
    }
    // Abaixo da zona morta o movimento deixa de ser suave e vira solavanco:
    // 0,46° por contagem de pulso a cada 20 ms dá 23 °/s de piso.
    if (velocidade < 23.0) {
      Serial.print(F("AVISO DEMO velocidade baixa "));
      Serial.println(velocidade, 1);
    }
    sweep_ativo = false;
    demo_ativo = true;
    demo_assentando = true;      // dispara o primeiro passo já no próximo laço
    demo_passo = 0;
    t_demo = millis() - DEMO_PAUSA_MS;
    Serial.print(F("OK DEMO "));
    Serial.print(DEMO_N);
    Serial.print(F(" passos a "));
    Serial.print(velocidade, 0);
    Serial.println(F(" graus/s"));

  } else if (strcmp(cmd, "SWEEP") == 0 && n == 4) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    sweep_de  = limitaAoIntervalo(id, atof(tok[2]));
    sweep_ate = limitaAoIntervalo(id, atof(tok[3]));
    sweep_id = id;
    sweep_ativo = true;
    defineAlvo(id, sweep_de);
    Serial.print(F("OK SWEEP "));
    Serial.print(id);
    Serial.print(' ');
    Serial.print(sweep_de, 1);
    Serial.print(' ');
    Serial.println(sweep_ate, 1);

  } else if (strcmp(cmd, "LIM") == 0 && n == 4) {
    int id = atoi(tok[1]);
    float a = atof(tok[2]);
    float b = atof(tok[3]);
    if (!idValido(id) || a > b) { erroArgs(texto); return; }
    lim_min[id] = a;
    lim_max[id] = b;
    defineAlvo(id, atual[id]);        // realinha o alvo se o novo limite o excluiu
    Serial.print(F("OK LIM "));
    Serial.print(id);
    Serial.print(' ');
    Serial.print(a, 1);
    Serial.print(' ');
    Serial.println(b, 1);

  } else if (strcmp(cmd, "LIM") == 0 && n == 1) {
    respondeLimites();

  } else if (strcmp(cmd, "SPEED") == 0 && n == 1) {
    Serial.print(F("OK SPEED "));
    Serial.println(velocidade, 1);

  } else if (strcmp(cmd, "SPEED") == 0 && n == 2) {
    float a = atof(tok[1]);
    // Teto de 150 °/s de MÉDIA porque a curva suavizada tem pico de 1,875x a
    // média: 150 vira 281 °/s no meio do movimento, que já é bastante para um
    // MG90S com carga. Acima disso o servo satura, não acompanha a curva, e a
    // suavização deixa de valer — o tranco volta.
    if (a <= 0 || a > 150) { erroArgs(texto); return; }
    velocidade = a;
    Serial.print(F("OK SPEED "));
    Serial.println(velocidade, 1);

  } else if (strcmp(cmd, "SPEEDG") == 0 && n == 1) {
    Serial.print(F("OK SPEEDG "));
    Serial.println(velocidade_garra, 1);

  } else if (strcmp(cmd, "SPEEDG") == 0 && n == 2) {
    float a = atof(tok[1]);
    if (a < 0 || a > VEL_GARRA_MAX) { erroArgs(texto); return; }
    velocidade_garra = a;
    Serial.print(F("OK SPEEDG "));
    Serial.print(velocidade_garra, 1);
    if (velocidade_garra <= 0.0) Serial.print(F(" (sem suavizacao)"));
    // Avisa quando o número pedido não muda mais nada: acima de ~500 o curso
    // inteiro da garra já cai no piso de duração, e quem pediu 900 esperando
    // o dobro de 450 merece saber que não vai ver diferença.
    if (velocidade_garra > 500.0) Serial.print(F(" (acima do piso, sem efeito pratico)"));
    Serial.println();

  } else if (strcmp(cmd, "DETACH") == 0 && n == 2 && strcmp(tok[1], "ALL") == 0) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) { ligado[i] = false; desligaSaida(i); }
    sweep_ativo = false;
    Serial.println(F("OK DETACH ALL"));

  } else if (strcmp(cmd, "DETACH") == 0 && n == 2) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    ligado[id] = false;
    desligaSaida(id);
    if (sweep_ativo && sweep_id == id) sweep_ativo = false;
    Serial.print(F("OK DETACH "));
    Serial.println(id);

  } else if (strcmp(cmd, "ATTACH") == 0 && n == 2 && strcmp(tok[1], "ALL") == 0) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      ligado[i] = true;
      alvo[i] = atual[i];
      duracao[i] = 0;
      escrevePulso(i, atual[i]);
    }
    Serial.println(F("OK ATTACH ALL"));

  } else if (strcmp(cmd, "ATTACH") == 0 && n == 2) {
    int id = atoi(tok[1]);
    if (!idValido(id)) { erroArgs(texto); return; }
    ligado[id] = true;
    alvo[id] = atual[id];             // religa onde está, não onde estava antes
    duracao[id] = 0;
    escrevePulso(id, atual[id]);
    Serial.print(F("OK ATTACH "));
    Serial.println(id);

  } else if (strcmp(cmd, "RAW") == 0 && n == 3) {
    int id = atoi(tok[1]);
    int pulso = atoi(tok[2]);
    if (!idValido(id) || pulso < 0 || pulso > 4095) { erroArgs(texto); return; }
    sweep_ativo = false;
    pwm.setPWM(CANAL_PCA[id], 0, (uint16_t)pulso);
    Serial.print(F("OK RAW "));
    Serial.print(id);
    Serial.print(' ');
    Serial.println(pulso);

  } else if (strcmp(cmd, "PULSO") == 0 && n == 4) {
    int id = atoi(tok[1]);
    int pi = atoi(tok[2]);
    int pj = atoi(tok[3]);
    if (!idValido(id) || pi < 0 || pj > 4095 || pi >= pj) { erroArgs(texto); return; }
    pulso_min[id] = pi;
    pulso_max[id] = pj;
    Serial.print(F("OK PULSO "));
    Serial.print(id);
    Serial.print(' ');
    Serial.print(pi);
    Serial.print(' ');
    Serial.println(pj);

  // ---- sensor de corrente -----------------------------------------------

  } else if (strcmp(cmd, "AMP") == 0 && n == 1) {
    Serial.print(F("AMP "));
    Serial.print(correnteAmp(), 3);
    Serial.print(' ');
    Serial.print(adcMedio(), 1);
    Serial.print(' ');
    Serial.println(acs_zero_adc, 1);

  } else if (strcmp(cmd, "ZERO") == 0 && n == 1) {
    medeZero();
    Serial.print(F("OK ZERO "));
    Serial.println(acs_zero_adc, 1);

  } else if (strcmp(cmd, "SENS") == 0 && n == 2) {
    float s = atof(tok[1]);
    if (s < 1.0 || s > 1000.0) { erroArgs(texto); return; }
    acs_mv_por_amp = s;
    Serial.print(F("OK SENS "));
    Serial.println(acs_mv_por_amp, 1);

  } else if (strcmp(cmd, "LIMAMP") == 0 && n == 2) {
    float a = atof(tok[1]);
    // Teto de 5 A: é o fundo de escala da versão mais sensível, e nenhum
    // servo de hobby chega perto disso sem já estar destruído.
    if (a < 0.0 || a > 5.0) { erroArgs(texto); return; }
    limite_stall_amp = a;
    em_alivio = false;
    stall_desistiu = false;
    Serial.print(F("OK LIMAMP "));
    Serial.println(limite_stall_amp, 2);

  } else if (strcmp(cmd, "LIMAMP") == 0 && n == 1) {
    Serial.print(F("LIMAMP "));
    Serial.print(limite_stall_amp, 2);
    Serial.print(limite_stall_amp > 0.0 ? F(" (ligado, canal ") : F(" (desligado, canal "));
    Serial.print(canal_medido);
    Serial.println(')');

  } else if (strcmp(cmd, "HELP") == 0 && n == 1) {
    mostraAjuda();

  } else {
    Serial.print(F("ERR UNKNOWN "));
    Serial.println(cmd);
  }
}

// ---------------------------------------------------------------------------

// Confere se o PCA9685 responde no barramento.
//
// A biblioteca não faz isso: pwm.begin() segue em frente com o barramento
// mudo, e daí em diante o firmware responde OK a todo SET enquanto nenhum
// servomotor se move. Um jumper solto no SDA vira "o firmware está com
// defeito", e o tempo vai embora procurando no lugar errado.
bool pcaResponde() {
  Wire.beginTransmission(0x40);
  return Wire.endTransmission() == 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!pcaResponde()) {
    Serial.println(F("ERR SEM_PCA9685"));
    Serial.println(F("# O modulo nao responde em 0x40. Os comandos serao aceitos"));
    Serial.println(F("# e nenhum servo vai se mover. Confira SDA no A4, SCL no A5,"));
    Serial.println(F("# VCC do PCA no 5V do Nano, e GND comum com a fonte."));
    Serial.println(F("# Para depurar a fiacao, use firmware/diagnostico/."));
  }

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  // Zero do sensor, medido com as saídas em "full off" — sem corrente alguma
  // passando pelo sensor. Medir depois de energizar deixava o zero uns 5
  // contagens alto, ou 130 mA de viés constante: não atrapalha a detecção de
  // travamento, mas estraga o AMP como medida.
  //
  // O preço é o braço ficar solto por ~550 ms antes de assumir a pose de
  // repouso. Ligue com ele já perto dela.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) desligaSaida(i);
  delay(50);
  medeZero();

  // Assume a pose de repouso como posição inicial em vez de ir até ela:
  // no boot não se sabe onde as juntas estão, e ramper a partir de um palpite
  // errado seria um movimento não comandado logo ao ligar.
  //
  // O limitaAoIntervalo aqui não é zelo: escrevePulso() só satura a contagem
  // em 0-4095 e quem conhece lim_min/lim_max é o defineAlvo, por onde o boot
  // não passa. Sem ele, uma pose_repouso fora dos limites vai crua contra o
  // batente, em velocidade máxima, a cada energização.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    float inicial = limitaAoIntervalo(i, pose_repouso[i]);
    if (fabs(inicial - pose_repouso[i]) > 0.05) {
      Serial.print(F("AVISO pose_repouso "));
      Serial.print(i);
      Serial.print(F(" em "));
      Serial.print(pose_repouso[i], 1);
      Serial.print(F(" fora de ["));
      Serial.print(lim_min[i], 0);
      Serial.print(',');
      Serial.print(lim_max[i], 0);
      Serial.print(F("], usando "));
      Serial.println(inicial, 1);
    }
    atual[i] = inicial;
    alvo[i] = inicial;
    ligado[i] = true;
    escrevePulso(i, atual[i]);
  }

  t_ultimo_passo = millis();

  Serial.println(F("READY braco_firmware"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      linha[n_linha] = '\0';
      if (n_linha > 0) processaComando(linha);
      n_linha = 0;
    } else if (c != '\r') {
      // Linha longa demais é descartada inteira em vez de truncada: metade de
      // um comando de ângulo é pior do que comando nenhum.
      if (n_linha < sizeof(linha) - 1) {
        linha[n_linha++] = c;
      } else {
        n_linha = 0;
        Serial.println(F("ERR TOO_LONG"));
      }
    }
  }

  atualizaMovimento();
  concluiImpulsoGarra();
  atualizaDemo();
  amostraCorrente();
  verificaStall();
}
