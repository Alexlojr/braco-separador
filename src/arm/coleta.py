"""
Orquestra a coleta: detecção do YOLO -> mm na mesa -> ângulos -> serial.

Este módulo é a cola entre visão e braço. Ele não abre câmera nem porta
serial: recebe as duas coisas prontas por injeção, o que deixa tudo
testável sem hardware (basta passar um mock com `.write()`).

A sequência de pega NÃO é a clássica de "top-down grasp", e o motivo é a
peça: ela tem um chapéu de 50 mm em cima de um tronco de 20 mm. Descer em
cima do tronco significa descer em cima do chapéu, que está no caminho.

Então a garra entra de lado e por baixo:

    abre -> sobrevoa ao LADO -> desce ao lado -> entra subindo ->
    fecha -> sobe -> leva -> solta

O "ao lado" é um recuo RADIAL: o mesmo ângulo de base, raio menor. Isso
importa porque deixa a base parada na etapa de entrada — só ombro e cotovelo
se movem, e a garra avança na direção em que ela abre, passando por fora do
chapéu em vez de por cima dele.

A entrada ainda sobe alguns milímetros no último trecho. Com a visão errando
até 2,2 mm, chegar subindo e chegar descendo têm consequências diferentes: o
erro para baixo empurra a peça contra a mesa ou a tomba, e o erro para cima
só fecha a garra um pouco acima do meio do tronco.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Iterator

from .kinematics import Braco, CalibracaoBraco, PoseInalcancavel, pose_para_servos


@dataclass
class ParametrosColeta:
    """Alturas e tempos da sequência. Ajuste quando a maquete existir."""
    z_mesa: float = 0.0          # altura da superfície, em mm
    z_pega: float = 15.0         # altura da garra ao fechar (meia altura do objeto)
    z_aproximacao: float = 70.0  # altura do sobrevoo antes de descer
    z_transporte: float = 80.0   # altura ao carregar o objeto

    # Altura em que a garra ABRE sobre o depósito. `None` solta na altura
    # de transporte, que era o comportamento até 10/11/2026.
    #
    # O ponto de pega é o meio do tronco, 25 mm acima da base da peça, então
    # soltar a 80 mm derruba a peça de 55 mm de altura — o bastante para ela
    # quicar e sair de uma placa de 100 mm. Descer antes transforma a
    # soltura em pousar.
    #
    # Suba se o depósito acumular peças: a garra desce até aqui e não sabe
    # o que já está embaixo.
    z_soltura: float | None = 25.0

    # Quanto a garra para ANTES da peça, medido no raio, para descer ao lado
    # dela em vez de em cima. Precisa ser maior que a metade do chapéu da
    # peça (25 mm), senão a descida acontece sobre ele.
    #
    # Não é livre: recuar encurta o raio, e o braço tem um raio mínimo. Em
    # 30 mm a sequência fecha em toda a área de coleta alcançável; acima
    # disso os alvos mais próximos da base começam a cair fora do envelope.
    # `planejar` reduz sozinho quando não couber, e diz na descrição da etapa
    # qual recuo usou.
    recuo_entrada: float = 30.0

    # Piso do recuo. Metade do chapéu da peça são 25 mm: abaixo disso a
    # descida ao lado acontece sobre o chapéu, e a sequência perde o sentido.
    # Alvo que não aceita este recuo é recusado, não aproximado na marra.
    recuo_minimo: float = 25.0

    # Quanto a garra sobe no trecho final, ao entrar no tronco. A descida ao
    # lado para em (z_pega - subida_entrada) e o último movimento é para
    # cima. Ver o cabeçalho do módulo.
    subida_entrada: float = 5.0

    # Correção do ponto que a visão entrega, ao longo do raio, afastando da
    # base. A homografia converte a BASE da bounding box, que numa vista
    # oblíqua é a borda do objeto virada para a câmera — não o centro do
    # apoio, que é onde a garra precisa fechar.
    #
    # Medido em 09/11/2026 com um mouse: o script relatou (-33, 137) mm; a
    # régua deu borda da frente em (-32,5 , 130) e centro em (-30, 150). O
    # erro ficou 7,0 mm da borda e 13,3 mm do centro, e praticamente TODO
    # em Y — 0,5 mm em X. Viés radial, como o modelo previa.
    #
    # A peça do projeto tem prato de 50 mm, que é a pegada dela na mesa, e o
    # centro fica meio prato mais longe no raio: 25 mm, no braco.json. Dez
    # vezes o erro da homografia e mais que a folga de 20 mm que a garra tem
    # de cada lado — sem corrigir, a pega erraria sempre para o mesmo lado.
    #
    # O padrão aqui continua 0 porque este valor é propriedade do OBJETO, não
    # do braço: um default diferente de zero deslocaria em silêncio qualquer
    # alvo de outro tamanho.
    deslocamento_radial: float = 0.0

    # Corta o PWM da base, do ombro e do cotovelo enquanto a garra abre ou
    # fecha, e religa antes do braço voltar a andar.
    #
    # Serve para a garra ter a fonte inteira no instante em que ela precisa.
    # O braço estendido é justamente onde a queda de tensão nos jumpers é
    # pior, e é onde a garra reclamava.
    #
    # Isso só é seguro porque o braço NÃO CAI quando solto: medido na
    # bancada em 09/11, com `DETACH 1` a ombro 160, estendido e com a peça
    # na garra. Quem segura é o atrito do redutor do servo, que nesses
    # micro-servos de plástico é alto o bastante para não ser retroacionável
    # com essa carga. É atrito, não freio: com peça mais pesada, com o braço
    # mais estendido, ou em minutos em vez de milissegundos, pode escorregar.
    # Por isso fica DESLIGADO por padrão e só vale para o intervalo curto em
    # que a garra age.
    # Pose de onde o braço parte depois do HOME, em graus de servo, na ordem
    # dos canais. Só serve para escolher a ordem dos SET da primeira pose do
    # ciclo — ver `_ordem_segura`. `None` deixa a primeira sem referência,
    # que é o pior caso e não quebra nada.
    pose_repouso: list | None = None

    soltar_na_pega: bool = False

    # Raio em que o braço gira a base, carregando a peça. `None` gira onde
    # estiver, que é o comportamento antigo.
    #
    # Girar esticado é o pior caso do sistema inteiro: o ombro está no maior
    # torque e a base tem a maior inércia para acelerar, tudo isso com três
    # juntas andando ao mesmo tempo e a garra segurando. No primeiro ciclo
    # real, em 09/11/2026, a base travava no meio do caminho e a sequência
    # seguia para soltar a peça ali mesmo — o firmware conta o tempo do
    # movimento, não a posição, então um servo que não acompanha passa por
    # movimento concluído.
    #
    # Recolher antes de girar tira os dois efeitos de uma vez, e é a mesma
    # regra que o pegada_manual.py já recomendava para a operação manual.
    # 130 mm deixa o ombro com ~79% do torque que ele teria a 165 mm, e está
    # com folga dentro da faixa alcançável na altura de transporte (114 a
    # 188 mm).
    raio_giro: float | None = 130.0

    # Corta o PWM do ombro e do cotovelo durante o giro da base.
    #
    # No giro eles não se movem: só seguram a pose, e quem segura de verdade
    # é o atrito do redutor — medido em 09/11, com DETACH a ombro 160 e a
    # peça na garra, nada se mexeu. Soltos, a corrente que eles gastavam
    # segurando sobra para a base, que é a única que precisa de torque ali.
    #
    # A GARRA NUNCA É SOLTA aqui: ela está segurando a peça.
    #
    # Diferente do soltar_na_pega, este intervalo tem movimento de verdade
    # acontecendo, e a aceleração lateral do giro faz força nas juntas
    # soltas. Por isso é separado, e por isso vale testar com peça antes de
    # confiar: se o braço ceder durante o giro, desligue.
    soltar_no_giro: bool = False

    # Solta TAMBÉM a garra durante o giro, deixando a base como único servo
    # energizado. É o que sobra quando soltar ombro e cotovelo não basta.
    #
    # Medido em 09/11/2026: a base percorre os 180° inteiros quando é a única
    # a consumir, e trava na metade quando as outras três seguram junto. Não
    # é obstáculo nem limite mecânico — é a tensão que chega nela.
    #
    # Soltar a garra COM A PEÇA parece temerário e não é, por dois motivos:
    # a retenção é por fechamento de forma, com os pratos de 50 mm impedindo
    # a peça de sair, e o redutor do servo não é retroacionável, então os
    # dedos ficam onde estão. Ainda assim é o ponto de maior risco da
    # sequência: se a peça cair durante o giro, é isto.
    soltar_garra_no_giro: bool = False

    # Fatia o giro da base em passos de no maximo tantos graus. `None` gira
    # de uma vez.
    #
    # Servo e controlador proporcional: o esforco acompanha o ERRO de
    # posicao. Um alvo 25 graus adiante satura o driver e pede corrente
    # maxima de uma vez; cinco alvos de 5 graus nunca saturam, e entre eles a
    # fonte recupera. Nao e o mesmo que adicionar atraso: muda o pico, nao so
    # o intervalo.
    #
    # Isso importa aqui porque a alimentacao dos servos passa por protoboard
    # e jumpers, cuja resistencia de contato vira queda de tensao no pico - e
    # torque de servo e quase proporcional a tensao. A correcao de fundo e
    # eletrica; esta e a que se faz por software.
    passo_giro: float | None = None
    # (x, y) do depósito, em mm. Precisa cair dentro dos limites da base:
    # um ponto "atrás" do braço (y negativo com x≈0) exige base ≈ -90°, que
    # nenhum servo de 0-180° alcança. Mantenha o depósito à frente ou ao lado.
    deposito: tuple[float, float] = (-80.0, 100.0)
    # Pausa DEPOIS de o braço chegar, para a estrutura assentar. Não é mais
    # a espera do movimento em si: quem espera é o `aguardar_parada` da
    # serial, que pergunta ao firmware quando ele de fato parou.
    pausa_s: float = 0.25

    # Pausa EXTRA depois das etapas de garra, somada à `pausa_s`.
    #
    # O firmware estima quanto a garra leva para chegar, porque ela não tem
    # realimentação: ninguém sabe onde ela está, só para onde foi mandada.
    # Se a estimativa for otimista, o `aguardar_parada` devolve cedo e a etapa
    # seguinte começa com a garra no meio do curso — foi o que aconteceu no
    # primeiro ciclo real, em 09/11: o braço subiu e a garra foi terminando de
    # fechar durante a subida.
    #
    # A correção de fundo é o VEL_MECANICA_GARRA do firmware. Esta pausa é a
    # margem que não exige recompilar, e que absorve a variação de velocidade
    # da garra com a tensão disponível — que muda conforme a pose do braço.
    pausa_garra_s: float = 0.5

    # Ângulos de servo da garra aberta e fechada. `None` usa o mínimo e o
    # máximo da junta, respectivamente.
    #
    # Estes valores são explícitos, e não deduzidos dos limites, por dois
    # motivos:
    #
    # 1. Qual extremo fecha a garra depende de como ela foi montada. Assumir
    #    "máximo fecha" é chute: montada ao contrário, a garra ABRE na hora de
    #    agarrar e o objeto nunca sai do lugar.
    # 2. Os objetos de teste são impressos com região de preensão dimensionada
    #    para um curso específico. Abrir sempre até o limite mecânico é mais
    #    do que o necessário, e fechar até o limite força o servo contra a
    #    peça depois de ela já estar retida pelo encaixe.
    garra_aberta: float | None = None
    garra_fechada: float | None = None


class SincroniaPerdida(RuntimeError):
    """
    O firmware parou de confirmar que terminou o movimento.

    Acontece quando a porta serial morre, quando o Arduino reinicia, ou
    quando o braco fica em movimento alem do tempo limite. Nos tres casos o
    lado Python perdeu o unico sinal que tinha de onde o braco esta.

    Isto PRECISA interromper a sequencia. O `aguardar_parada` devolve False
    em vez de travar - decisao certa - mas o valor estava sendo descartado, e
    o resultado era a sequencia inteira disparando de uma vez, sem espera
    nenhuma entre etapas. Na bancada isso apareceu como a base parando no
    meio do giro: os seis passos chegaram em milissegundos.
    """


@dataclass
class Etapa:
    descricao: str
    comandos: list[str]


class ControladorColeta:
    def __init__(
        self,
        serial,
        braco: Braco | None = None,
        calib: CalibracaoBraco | None = None,
        params: ParametrosColeta | None = None,
    ):
        self.serial = serial
        self.braco = braco or Braco()
        self.calib = calib or CalibracaoBraco()
        self.params = params or ParametrosColeta()

    # ---- montagem da sequência (sem efeito colateral) --------------------

    def _cmds_pose(self, x: float, y: float, z: float,
                   anterior: dict | None = None) -> list[str]:
        canais = pose_para_servos(self.braco, self.calib, x, y, z)
        ordem = self._ordem_segura(canais, anterior)
        if anterior is not None:
            anterior.update(canais)
        return [f"SET {canal} {canais[canal]}" for canal in ordem]

    def _ordem_segura(self, canais: dict, anterior: dict | None) -> list[int]:
        """
        Em que ordem mandar os SET para o firmware não saturar o acoplamento.

        O firmware avalia o limite acoplado contra o ALVO da outra junta
        quando as duas estão no mesmo lote — mas o PRIMEIRO comando do lote
        chega quando a outra ainda não entrou, e aí ele usa a posição dela.
        Se o par (primeiro novo, outro ainda parado) violar o limite, o
        firmware satura e o braço vai para outra pose, dizendo ACOPLADO.

        Na bancada isso tirava até 7,6° do ombro nas poses de trânsito.

        Quase sempre uma das duas ordens é segura, porque só uma das pontas
        do "cotovelo" do caminho cai na região proibida. Então: testa as
        duas, manda primeiro a que passa. Se nenhuma passar, a ordem não
        resolve e o firmware satura de qualquer jeito — o que é o
        comportamento correto, porque aí a transição é mesmo insegura.
        """
        ombro, cotovelo = self.calib.ombro.canal, self.calib.cotovelo.canal
        base = self.calib.base.canal
        padrao = sorted(canais)

        if anterior is None or ombro not in canais or cotovelo not in canais:
            return padrao
        if ombro not in anterior or cotovelo not in anterior:
            return padrao

        def seguro(primeiro, parado):
            """O par (novo do primeiro, antigo do parado) respeita o limite?"""
            s1 = canais[ombro] if primeiro == ombro else anterior[ombro]
            s2 = canais[cotovelo] if primeiro == cotovelo else anterior[cotovelo]
            return self.calib.viola_acoplamento(s1, s2) is None

        if seguro(ombro, cotovelo):
            return [base, ombro, cotovelo] if base in canais else [ombro, cotovelo]
        if seguro(cotovelo, ombro):
            return [base, cotovelo, ombro] if base in canais else [cotovelo, ombro]
        return padrao

    def _ponto_recuado(self, x: float, y: float, recuo: float):
        """
        O mesmo ângulo de base, com o raio encurtado em `recuo`.

        Manter o ângulo é o ponto todo: a etapa de entrada fica sem mexer a
        base, e a garra avança exatamente na direção em que os dedos abrem.
        """
        r = math.hypot(x, y)
        if r <= 1e-6:
            return x, y
        fator = max(0.0, r - recuo) / r
        return x * fator, y * fator

    def _alcanca(self, x: float, y: float, z: float) -> bool:
        try:
            pose_para_servos(self.braco, self.calib, x, y, z)
            return True
        except PoseInalcancavel:
            return False

    def ponto_de_pega(self, x: float, y: float):
        """
        O ponto da visão corrigido para o centro do apoio.

        Afasta da base por `deslocamento_radial`, ao longo do raio. Radial e
        não em Y porque o viés acompanha a direção de visada, e com a câmera
        atrás da base essa direção é a radial — foi o que a medição de
        09/11 mostrou, com o erro quase todo em Y para um alvo quase em
        frente à base, e X errando 0,5 mm.
        """
        if not self.params.deslocamento_radial:
            return x, y
        return self._ponto_recuado(x, y, -self.params.deslocamento_radial)

    def _recuo_viavel(self, x: float, y: float, z: float) -> float:
        """
        O maior recuo que o braço alcança, entre o configurado e o mínimo.

        Recuar encurta o raio, e perto da base o envelope é estreito: o ponto
        de descida pode cair fora dele mesmo com o ponto de pega dentro. Em
        vez de recusar o alvo, usa o recuo que couber.

        O piso é `recuo_minimo`, e ele não é negociável: abaixo da metade do
        chapéu a descida acontece sobre o chapéu, que é exatamente o que esta
        sequência existe para evitar. Alvo que não aceita o mínimo é recusado.
        """
        passo = 5.0
        recuo = self.params.recuo_entrada
        while recuo >= self.params.recuo_minimo:
            if self._alcanca(*self._ponto_recuado(x, y, recuo), z):
                return recuo
            recuo -= passo
        raise PoseInalcancavel(
            f"alvo em ({x:.0f}, {y:.0f}) é alcançável, mas o ponto de entrada "
            f"lateral não: com o recuo mínimo de {self.params.recuo_minimo:.0f} mm "
            f"o braço teria de chegar a {math.hypot(x, y) - self.params.recuo_minimo:.0f} mm "
            f"do eixo, a {z:.0f} mm de altura, e ali ele não dobra o suficiente. "
            f"Recuar menos faria a garra descer sobre o chapéu da peça.",
            motivo=PoseInalcancavel.ENVELOPE,
        )

    def _z_sobrevoo(self, rx: float, ry: float, z_entrada: float) -> float | None:
        """
        A maior altura alcançável sobre o ponto de descida, até z_aproximacao.

        `None` quando não há nenhuma acima da própria descida — aí o sobrevoo
        não existe e a sequência vai direto para o ponto de descida.

        Perto da base o envelope é um corredor estreito: num raio de 100 mm o
        braço só alcança de 20 a 40 mm de altura. Exigir sobrevoo fixo a
        70 mm ali recusaria o alvo, e o sobrevoo não precisa ser alto — ele
        só serve para a garra chegar ao ponto de descida de cima, em vez de
        varrer a mesa de lado.
        """
        z = self.params.z_aproximacao
        while z > z_entrada:
            if self._alcanca(rx, ry, z):
                return z
            z -= 5.0
        return None

    def _cmd_garra(self, fechada: bool) -> str:
        g = self.calib.garra
        p = self.params

        if fechada:
            angulo = p.garra_fechada if p.garra_fechada is not None else g.maximo
        else:
            angulo = p.garra_aberta if p.garra_aberta is not None else g.minimo

        # Um ângulo de garra fora dos limites da junta é erro de configuração,
        # não pose inalcançável — mas falha do mesmo jeito, e antes de mover.
        if not g.dentro_dos_limites(angulo):
            raise PoseInalcancavel(
                f"Garra {'fechada' if fechada else 'aberta'} em {angulo:.1f}°, "
                f"fora do limite [{g.minimo:.0f}, {g.maximo:.0f}]. "
                f"Confira 'garra_aberta_graus'/'garra_fechada_graus' na config.",
                motivo=PoseInalcancavel.LIMITE_JUNTA,
            )

        return f"SET {g.canal} {round(angulo, 1)}"

    def planejar(
        self,
        x: float,
        y: float,
        deposito: tuple[float, float] | None = None,
    ) -> list[Etapa]:
        """
        Monta a sequência completa para pegar em (x, y) e levar ao depósito.

        Levanta PoseInalcancavel ANTES de mover qualquer servo se alguma
        pose da sequência for impossível — validar o plano inteiro antes de
        executar evita o braço parar no meio do movimento com o objeto preso
        na garra, que é bem pior de recuperar do que simplesmente recusar.
        """
        p = self.params
        # `deposito` sobrepoe o da configuracao para esta sequencia so.
        # E assim, e nao trocando o parametro do controlador, porque o
        # destino depende da CLASSE detectada e muda a cada ciclo: um
        # controlador por deposito teria duas copias de toda a
        # calibracao, que e justamente o que nao pode divergir.
        dx, dy = p.deposito if deposito is None else deposito

        # A visão entrega a borda do objeto, não o centro do apoio. Corrigir
        # aqui, antes de tudo, faz o recuo e o sobrevoo saírem do ponto certo.
        x, y = self.ponto_de_pega(x, y)

        # Acompanha a pose comandada ao longo da sequência, para `_cmds_pose`
        # poder escolher a ordem dos SET. Parte da pose de repouso, que é
        # onde o HOME deixou o braço no fim do ciclo anterior.
        pose = ({i: ang for i, ang in enumerate(p.pose_repouso)}
                if p.pose_repouso else {})

        # Valida o ponto de pega ANTES de procurar o recuo: se o alvo não é
        # alcançável, o erro tem que falar do alvo, e não do ponto de entrada.
        # Sem `pose`, porque aqui é só validação — os comandos de verdade são
        # gerados na ordem certa lá embaixo, quando a pose anterior já é
        # conhecida.
        self._cmds_pose(x, y, p.z_pega)

        z_entrada = p.z_pega - p.subida_entrada
        recuo = self._recuo_viavel(x, y, z_entrada)
        rx, ry = self._ponto_recuado(x, y, recuo)
        z_alto = self._z_sobrevoo(rx, ry, z_entrada)

        etapas = [Etapa("Abrindo a garra", [self._cmd_garra(False)])]

        if z_alto is not None:
            etapas.append(Etapa(
                f"Sobrevoando {recuo:.0f} mm antes do alvo, a {z_alto:.0f} mm",
                self._cmds_pose(rx, ry, z_alto, pose)))

        # DETACH antes de a garra agir, ATTACH antes de o braço voltar a andar.
        # Os dois andam sempre em par: uma etapa que solta tem de ser seguida,
        # mais cedo ou mais tarde, por uma que religa, senão o braço fica mole
        # no meio da sequência.
        canais = [self.calib.base.canal, self.calib.ombro.canal,
                  self.calib.cotovelo.canal]
        soltar = [f"DETACH {c}" for c in canais] if p.soltar_na_pega else []
        religar = [f"ATTACH {c}" for c in canais] if p.soltar_na_pega else []

        etapas += [
            Etapa(f"Descendo ao lado da peça, a {z_entrada:.0f} mm",
                  self._cmds_pose(rx, ry, z_entrada, pose)),
            Etapa("Entrando no tronco, por baixo do chapéu",
                  self._cmds_pose(x, y, p.z_pega, pose)),
            Etapa("Fechando a garra", soltar + [self._cmd_garra(True)]),
            Etapa("Subindo com o objeto",
                  religar + self._cmds_pose(x, y, p.z_transporte, pose)),
        ]
        etapas += self._etapas_transporte(x, y, dx, dy, pose)
        etapas += [
            *([Etapa(f"Descendo sobre o depósito, a {p.z_soltura:.0f} mm",
                     self._cmds_pose(dx, dy, p.z_soltura, pose))]
              if p.z_soltura is not None else []),
            Etapa("Soltando o objeto", soltar + [self._cmd_garra(False)]),
            Etapa("Voltando para HOME", religar + ["HOME"]),
        ]
        return etapas

    def _etapas_transporte(self, x, y, dx, dy, pose=None):
        """
        Do alvo até o depósito, recolhendo antes de girar a base.

        Três movimentos curtos em vez de um longo: recolhe no mesmo ângulo,
        gira só a base, estende sobre o depósito. Cada um move menos juntas e
        pede menos corrente de pico que o movimento único, que combinava giro
        de base, mudança de altura e mudança de alcance ao mesmo tempo.

        Com `raio_giro = None` volta ao movimento único.
        """
        p = self.params
        z = p.z_transporte

        if p.raio_giro is None:
            return [Etapa("Levando ao depósito",
                          self._cmds_pose(dx, dy, z, pose))]

        perto_alvo = self._no_raio(x, y, p.raio_giro)
        perto_dep = self._no_raio(dx, dy, p.raio_giro)

        cmds_recolhido = self._cmds_pose(*perto_alvo, z, pose)
        cmds_girado = self._cmds_pose(*perto_dep, z, pose)

        # Só o que MUDA. Recolhido e girado estão no mesmo raio e na mesma
        # altura, então ombro e cotovelo repetem o mesmo ângulo e os comandos
        # deles seriam ruído — e, com eles soltos, comando para junta sem PWM.
        so_a_base = [c for c in cmds_girado if c not in cmds_recolhido]

        # A base nunca entra: é ela que gira. Soltar e religar a base seria
        # perder e recuperar a referência de posição dela sem ganhar nada.
        sem_pwm = []
        if p.soltar_no_giro:
            sem_pwm += [self.calib.ombro.canal, self.calib.cotovelo.canal]
        if p.soltar_garra_no_giro:
            sem_pwm.append(self.calib.garra.canal)

        soltar = [f"DETACH {c}" for c in sem_pwm]
        religar = [f"ATTACH {c}" for c in sem_pwm]

        etapas = [Etapa(f"Recolhendo para girar, a {p.raio_giro:.0f} mm",
                        cmds_recolhido)]
        etapas += self._etapas_giro(cmds_recolhido, so_a_base, soltar)
        etapas.append(Etapa("Estendendo sobre o depósito",
                            religar + self._cmds_pose(dx, dy, z, pose)))
        return etapas

    def _etapas_giro(self, cmds_recolhido, so_a_base, soltar):
        """
        O giro da base, de uma vez ou fatiado em `passo_giro` graus.

        Fatiar reduz o PICO de corrente, e não só o intervalo: o servo
        aciona proporcionalmente ao erro de posição, então um alvo 25° à
        frente satura o driver e cinco alvos de 5° não.
        """
        canal = self.calib.base.canal
        passo = self.params.passo_giro

        if passo is None or not so_a_base:
            return [Etapa("Girando a base para o depósito", soltar + so_a_base)]

        prefixo = f"SET {canal} "
        inicio = next(float(c[len(prefixo):]) for c in cmds_recolhido
                      if c.startswith(prefixo))
        fim = next(float(c[len(prefixo):]) for c in so_a_base
                   if c.startswith(prefixo))

        curso = fim - inicio
        n = max(1, math.ceil(abs(curso) / passo))

        etapas = []
        for k in range(1, n + 1):
            angulo = inicio + curso * k / n
            etapas.append(Etapa(
                f"Girando a base, {k}/{n}",
                (soltar if k == 1 else []) + [f"SET {canal} {round(angulo, 1)}"],
            ))
        return etapas

    def _no_raio(self, x: float, y: float, raio: float):
        """O mesmo ângulo de base, no raio pedido."""
        r = math.hypot(x, y)
        if r <= 1e-6:
            return 0.0, raio
        return x * raio / r, y * raio / r

    def alcancavel(self, x: float, y: float) -> tuple[bool, str]:
        """Testa se dá para coletar em (x, y) sem mover nada."""
        try:
            self.planejar(x, y)
            return True, "alcançável"
        except PoseInalcancavel as e:
            return False, str(e)

    # ---- execução --------------------------------------------------------

    def executar(
        self,
        x: float,
        y: float,
        ao_progredir: Callable[[int, int, str], None] | None = None,
        deposito: tuple[float, float] | None = None,
    ) -> Iterator[Etapa]:
        """
        Executa a coleta, rendendo cada etapa conforme acontece.

        É um gerador para que a interface possa mostrar progresso em tempo
        real sem que este módulo precise saber que o Streamlit existe.
        """
        # Valida tudo antes de mover, DESTINO INCLUIDO: deposito fora
        # de alcance recusa a coleta sem a peca sair do lugar, em vez
        # de parar no meio do transporte com ela na garra.
        etapas = self.planejar(x, y, deposito)
        concluiu = False

        try:
            yield from self._executar_etapas(etapas, ao_progredir)
            concluiu = True
        finally:
            # Saída anormal entre um DETACH e o ATTACH correspondente deixaria
            # o braço mole, provavelmente no ar e com a peça. Vale para Ctrl-C,
            # para a sincronia perdida e para qualquer exceção.
            #
            # Só na saída anormal: a sequência completa já religa tudo no
            # caminho, e mandar de novo poluiria o fim de toda coleta com um
            # comando que não faz nada.
            if not concluiu and (self.params.soltar_na_pega
                                 or self.params.soltar_no_giro
                                 or self.params.soltar_garra_no_giro):
                self.serial.write(b"ATTACH ALL\n")

    def _executar_etapas(self, etapas, ao_progredir):
        total = len(etapas)

        for i, etapa in enumerate(etapas, start=1):
            if ao_progredir:
                ao_progredir(i, total, etapa.descricao)
            for cmd in etapa.comandos:
                self.serial.write((cmd + "\n").encode())

            # Espera o movimento TERMINAR, não um tempo fixo.
            #
            # O firmware suaviza cada movimento, então a duração depende da
            # distância percorrida: fechar a garra (100°) leva quase o triplo
            # de descer até o objeto (33°). Uma pausa fixa erra nos dois
            # sentidos, e o erro perigoso é o curto: a etapa seguinte começa
            # com a anterior pela metade, e a garra sobe antes de ter fechado.
            aguardar = getattr(self.serial, "aguardar_parada", None)
            if aguardar is not None and not aguardar():
                falhas = getattr(self.serial, "falhas_envio", 0)
                raise SincroniaPerdida(
                    f"o firmware parou de confirmar o fim do movimento na "
                    f"etapa {etapa.descricao!r}"
                    + (f"; {falhas} comando(s) falharam ao sair pela serial"
                       if falhas else "")
                    + ". Sem essa confirmacao nao da para saber onde o braco "
                      "esta, e seguir mandando pose seria chutar."
                )

            # Depois de chegar, uma pausa curta para o braço assentar: o servo
            # para no ângulo, mas a estrutura de acrílico ainda oscila um
            # pouco. Vale principalmente antes de fechar a garra.
            time.sleep(self.params.pausa_s)

            # A garra ganha um tempo a mais, porque o fim do movimento dela é
            # estimado e não medido. Ver `pausa_garra_s`.
            if self._mexe_a_garra(etapa):
                time.sleep(self.params.pausa_garra_s)

            yield etapa

    def _mexe_a_garra(self, etapa) -> bool:
        """Esta etapa manda algum SET no canal da garra?"""
        prefixo = f"SET {self.calib.garra.canal} "
        return any(cmd.startswith(prefixo) for cmd in etapa.comandos)

    # ---- entrada vinda da visão -----------------------------------------

    def coletar_deteccao(self, homografia, caixa, ao_progredir=None) -> Iterator[Etapa]:
        """
        Ponta a ponta: bounding box do YOLO -> coleta executada.

        `caixa` é (x1, y1, x2, y2) em pixels, como o YOLO devolve.
        """
        x_mm, y_mm = homografia.caixa_para_mm(*caixa)
        yield from self.executar(x_mm, y_mm, ao_progredir=ao_progredir)
