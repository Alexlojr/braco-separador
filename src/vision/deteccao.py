"""
Camada fina sobre o YOLO: escolher a detecção que interessa e desenhá-la.

Existe porque essa lógica estava copiada em três lugares — a dashboard, o
visualizador standalone e o pipeline de coleta. Três cópias da mesma regra
significam que ajustar o critério de confiança num deles e esquecer dos
outros faz o mesmo objeto ser aceito por um caminho e recusado por outro,
com o pipeline inteiro parecendo intermitente.

O critério é sempre o mesmo: uma detecção por frame, a de maior confiança
acima do limiar. O braço só pega um objeto de cada vez, então rastrear vários
não acrescentaria nada — e escolher "o mais confiante" é mais previsível do
que "o primeiro da lista", que depende da ordem interna do modelo.
"""

from __future__ import annotations

from dataclasses import dataclass

# Fallback para quando o braco.json não tem a seção "visao". Classes COCO,
# provisórias até o modelo ser treinado nos objetos do projeto: 67 = celular.
#
# Filtrar classe NÃO alivia o processamento. O `classes=` do Ultralytics filtra
# DEPOIS da inferência: a rede roda inteira de qualquer jeito, com as 80
# saídas, e o filtro só descarta caixas no fim. O que muda o custo é a
# resolução de entrada (`imgsz`), que é quadrática — 320 custa um quarto
# de 640.
CLASSE_ALVO_PADRAO = "cell phone"
CONFIANCA_MINIMA = 0.5

# Compatibilidade: havia uma lista [pessoa, celular] usada direto pelo painel
# e pelo visualizador, enquanto a coleta usava outra coisa. Era o painel
# procurando celular enquanto a bancada testava com mouse, sem nada avisando.
CLASSES_PERMITIDAS = [0, 67]

class ClasseDesconhecida(ValueError):
    """Pedido de classe que não existe no modelo. A mensagem lista parecidas."""


def resolver_classe(pedido, nomes) -> int:
    """
    Aceita "mouse", "MOUSE" ou "64" e devolve o número da classe.

    Nome em vez de número porque trocar o objeto de teste é frequente, e
    decorar que celular é 67 e mouse é 64 não ajuda ninguém. Pedido inválido
    levanta ClasseDesconhecida com as parecidas na mensagem, em vez de rodar
    meia hora detectando a classe errada.
    """
    texto = str(pedido).strip()

    if texto.lstrip("-").isdigit():
        numero = int(texto)
        if numero in nomes:
            return numero
        raise ClasseDesconhecida(
            f"não existe a classe {numero}: o modelo vai de 0 a {max(nomes)}."
        )

    procurado = texto.lower()
    for numero, nome in nomes.items():
        if nome.lower() == procurado:
            return numero

    parecidas = [f"{n} ({i})" for i, n in nomes.items() if procurado in n.lower()]
    dica = ("Parecidas: " + ", ".join(parecidas[:8]) if parecidas
            else "Pequenas e boas para teste: mouse (64), remote (65), "
                 "cell phone (67), scissors (76), book (73), cup (41).")
    raise ClasseDesconhecida(f"nenhuma classe chamada {texto!r}. {dica}")


def carregar_preferencias(caminho="config/braco.json"):
    """
    (classe_alvo, confianca_minima) da seção "visao" do braco.json.

    Existe para haver UM lugar dizendo o que o sistema procura. Antes a
    coleta tinha a classe por linha de comando e o painel tinha a lista fixa
    no código: trocar o objeto de teste na coleta deixava o painel procurando
    outra coisa, sem caixa nenhuma na tela e sem nada explicando por quê.

    A classe sai como veio do arquivo — nome ou número. Quem usa resolve com
    `resolver_classe`, que precisa do `model.names`.
    """
    import json
    from pathlib import Path

    try:
        dados = json.loads(Path(caminho).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return CLASSE_ALVO_PADRAO, CONFIANCA_MINIMA

    secao = dados.get("visao", {})
    return (
        secao.get("classe_alvo", CLASSE_ALVO_PADRAO),
        float(secao.get("confianca_minima", CONFIANCA_MINIMA)),
    )


MODELO_PADRAO = "models/yolov8n.pt"


def caminho_modelo(caminho_config, base_dir):
    """
    Caminho dos pesos do YOLO, da secao "visao" do braco.json.

    Um lugar so, pelo mesmo motivo da classe_alvo: coleta, painel e
    camera tinham cada um o seu caminho fixo no codigo, e trocar os pesos
    base pelos treinados exigiria editar tres arquivos. Esquecer um deixa
    o painel desenhando caixas de um modelo enquanto a coleta age por
    outro - sem erro nenhum, so com comportamento que nao bate.

    Caminho relativo e resolvido contra a RAIZ DO PROJETO, nao contra o
    diretorio corrente: painel e coleta sao iniciados de lugares
    diferentes.

    Arquivo inexistente levanta FileNotFoundError aqui, com o caminho
    resolvido na mensagem. Deixar passar para o Ultralytics e pior: com
    nome de peso conhecido ele tenta BAIXAR da internet, e com caminho
    errado devolve um erro que nao diz de onde o caminho veio.
    """
    import json
    from pathlib import Path

    try:
        dados = json.loads(Path(caminho_config).read_text(encoding="utf-8"))
        pedido = dados.get("visao", {}).get("modelo") or MODELO_PADRAO
    except (OSError, ValueError):
        pedido = MODELO_PADRAO

    caminho = Path(pedido)
    if not caminho.is_absolute():
        caminho = Path(base_dir) / caminho
    if not caminho.exists():
        raise FileNotFoundError(
            f"pesos do YOLO nao encontrados em {caminho} (visao.modelo = "
            f"{pedido!r} no braco.json). Depois do treino eles ficam em "
            f"runs/detect/train*/weights/best.pt."
        )
    return caminho


# Roda o YOLO a cada N frames. Entre inferências o desenho reaproveita a
# última caixa, o que mantém a imagem fluida sem custar inferência.
INTERVALO_DETECCAO = 3


@dataclass
class Deteccao:
    x1: float
    y1: float
    x2: float
    y2: float
    classe: int
    confianca: float
    nome: str

    @property
    def caixa(self) -> tuple[float, float, float, float]:
        """No formato que a homografia espera."""
        return (self.x1, self.y1, self.x2, self.y2)

    @property
    def rotulo(self) -> str:
        return f"{self.nome} {self.confianca:.2f}"


def melhor_bruta(resultado, nomes):
    """
    A detecção de maior confiança, SEM aplicar limiar. None se não houver.

    Existe para o diagnóstico: "o modelo não viu nada" e "o modelo viu com
    0,38 de confiança e o limiar é 0,50" pedem providências opostas, e as
    duas chegavam como `melhor_deteccao() is None`.
    """
    melhor = None
    maior = 0.0

    for caixa in resultado.boxes:
        conf = float(caixa.conf[0])
        if conf > maior:
            maior = conf
            melhor = caixa

    if melhor is None:
        return None

    x1, y1, x2, y2 = (float(v) for v in melhor.xyxy[0])
    classe = int(melhor.cls[0])
    return Deteccao(x1, y1, x2, y2, classe, maior, nomes[classe])


def melhor_deteccao(resultado, nomes, confianca_minima: float = CONFIANCA_MINIMA):
    """
    A detecção de maior confiança acima do limiar, ou None.

    `resultado` é um item do que o Ultralytics devolve (results[0]); `nomes`
    é o `model.names`.
    """
    d = melhor_bruta(resultado, nomes)
    if d is None or d.confianca < confianca_minima:
        return None
    return d


def desenhar(frame, deteccao: Deteccao, cor=(0, 255, 0)):
    """Desenha a caixa e o rótulo no frame, no lugar (modifica o array)."""
    import cv2

    x1, y1, x2, y2 = (int(v) for v in deteccao.caixa)
    cv2.rectangle(frame, (x1, y1), (x2, y2), cor, 2)
    cv2.putText(
        frame,
        deteccao.rotulo,
        (x1, y1 - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        cor,
        2,
    )
    return frame


def para_rgb(frame):
    """
    Converte o frame da Picamera2 para RGB.

    O formato "RGB888" da Picamera2 na verdade entrega os bytes em ordem BGR.
    Sem esta conversão, st.image() (que espera RGB) mostra tudo azulado.
    """
    import cv2

    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
