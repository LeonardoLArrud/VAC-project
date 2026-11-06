# Como rodar a simulação no Webots (Windows, macOS e Linux)

A simulação usa a mesma FSM do robô real: segue a linha, detecta as interseções, para diante de obstáculos e
navega na grade 5x5 com Dijkstra (`Robo_FSM/graph/`). Há três formas de rodar:

| Modo | Para quê | Precisa de |
|---|---|---|
| [1. Pelo Webots](#1-pelo-webots-o-mais-simples) | ver o robô andar, com ou sem missão | Webots + `numpy` |
| [2. Entregas pelo terminal](#2-entregas-digitadas-no-terminal-cli_webotspy) | digitar destinos enquanto o robô anda | Webots + `numpy` + `rich` |
| [3. Offline](#3-offline-sem-webots) | rodar os 22 cenários de teste sem abrir o Webots | `numpy` + `opencv-python` (+ `matplotlib`) |

Nos caminhos abaixo, `<repo>` é a pasta onde você clonou o projeto e `SIM` é
`<repo>/Webots/Simulation_Webots/Simulation_Webots`.

## 0. Instalação (uma vez)

### Webots

Instale o **Webots R2025a**, a versão dos mundos: <https://cyberbotics.com>.

| SO | Pasta de instalação padrão (`WEBOTS_HOME`) |
|---|---|
| Windows | `C:\Program Files\Webots`, ou `%LOCALAPPDATA%\Programs\Webots` se instalou só para o seu usuário |
| macOS | `/Applications/Webots.app` |
| Linux (.deb ou .tar.bz2) | `/usr/local/webots` |

### Python e pacotes

Python 3 recente (testado com o 3.11). Instale os pacotes **no mesmo Python que o Webots vai usar**:

| SO | Comando |
|---|---|
| Windows | `python -m pip install numpy rich opencv-python matplotlib` |
| macOS | `python3 -m pip install numpy rich opencv-python matplotlib` |
| Linux (Ubuntu/Debian) | `sudo apt install python3-numpy python3-rich python3-opencv python3-matplotlib` |

Para só ver a simulação (modo 1), basta o `numpy`. O `rich` é para o modo 2, e o `opencv-python` e o
`matplotlib` são para o modo 3.

No Ubuntu/Debian recente, o `pip install` fora de um ambiente virtual é bloqueado: use o `apt` acima ou
um `venv` (`python3 -m venv .venv`, depois `source .venv/bin/activate` e o `pip install`).

### Dizer ao Webots qual Python usar

No Webots: **Tools → Preferences → General → Python command**.

- Windows: `python`
- macOS e Linux: `python3`
- Com um `venv`, coloque o caminho completo do Python do `venv` (por exemplo
  `<repo>/.venv/bin/python3`, ou `<repo>\.venv\Scripts\python.exe` no Windows).

## 1. Pelo Webots (o mais simples)

1. *(Opcional)* Escolha a missão em `SIM/controllers/fsm/fsm.py`, na linha `MISSION = ...`:
   - `MISSION = None`: segue a linha e vai reto nas interseções.
   - `MISSION = ("MISSION", (0, 0), "E", (4, 3))`: sai do nó (0, 0) olhando para o leste e vai até o nó (4, 3).
     x cresce para a direita, y para cima, e (0, 0) é o canto de baixo à esquerda.
2. No Webots: **File → Open World →** `SIM/worlds/grid_5x5.wbt`.
3. Aperte **▶**. O robô começa sozinho 1 s depois.

Teclas, com a vista 3D clicada:

| Tecla | Faz |
|---|---|
| `G` | começa |
| `S` ou espaço | para |
| `M` | manda de novo o `MISSION` do `fsm.py` |
| `L` | tampa e destampa a câmera |

Enquanto roda:

- o console do Webots mostra cada transição (`[FSM] t= 8.64s FOLLOW -> WAIT ...`) e cada interseção (`[NODE] ...`);
- <http://localhost:8000/> mostra a câmera e os pontos da linha, e <http://localhost:8000/mask> o que o seguidor enxerga.

Outros mundos da pasta `SIM/worlds/`:

| Mundo | O que tem |
|---|---|
| `grid_5x5.wbt` | grade 5x5 limpa, 0,50 m entre os nós |
| `grid_5x5_box.wbt` | uma caixa entre (2, 3) e (3, 3): com a missão acima, o robô para, volta e recalcula a rota |
| `grid_5x5_reflexos.wbt` | reflexos brancos no chão, para testar o filtro |
| `huit.wbt` | a pista em 8. Roda o controlador antigo `basique`; para usar a FSM, mude na árvore da cena: `Cherokey_4W_v3 → controller → "fsm"` |

Antes de cada nova rodada: **⟲ (reset)** e **▶**.

## 2. Entregas digitadas no terminal (`cli_webots.py`)

O robô fica parado e você digita os destinos num terminal. Usa os mundos `*_extern.wbt`, em que o Webots
espera os controladores conectarem pelo terminal.

1. No Webots, abra `SIM/worlds/grid_5x5_extern.wbt` (ou `grid_5x5_box_extern.wbt`, com a caixa) e aperte **▶**.
2. Num terminal, rode os comandos do seu sistema:

**Windows (PowerShell)**

```powershell
$env:WEBOTS_HOME = "C:\Program Files\Webots"      # ou "$env:LOCALAPPDATA\Programs\Webots"
cd <repo>\Webots\Simulation_Webots\Simulation_Webots\controllers\fsm
& "$env:WEBOTS_HOME\msys64\mingw64\bin\webots-controller.exe" --robot-name=Cherokey_4W_v3 cli_webots.py --lp
```

No PowerShell, o `&` antes do caminho entre aspas é obrigatório.

**macOS**

```bash
export WEBOTS_HOME=/Applications/Webots.app
cd <repo>/Webots/Simulation_Webots/Simulation_Webots/controllers/fsm
"$WEBOTS_HOME/Contents/MacOS/webots-controller" --robot-name=Cherokey_4W_v3 cli_webots.py --lp
```

**Linux**

```bash
export WEBOTS_HOME=/usr/local/webots
cd <repo>/Webots/Simulation_Webots/Simulation_Webots/controllers/fsm
"$WEBOTS_HOME/webots-controller" --robot-name=Cherokey_4W_v3 cli_webots.py --lp
```

3. Ele pergunta onde o robô começa:
   - `Start node x y`: o nó, por exemplo `2 1`;
   - `Heading`: para onde ele olha, `E`, `N`, `W` ou `S`.

   Enter mantém (0, 0) olhando para o leste, que é onde o mundo coloca o robô. O `grid_pose` leva o robô até
   esse nó, nessa direção, assim que a simulação começa.
4. Quando aparecer `connected to Webots`, digite um destino `x y` (de 0 a 4) e Enter, por exemplo `4 3`.
   Na chegada ele pede o próximo destino, a partir de onde parou.
   - `q` + Enter encerra e mostra o resumo das entregas.
   - Com `--lp`, um painel mostra a rota, os obstáculos encontrados (✗) e o carro enquanto ele anda.
     Nesse momento, só `q` funciona, sozinho e sem Enter, e para o robô. Sem `--lp`, as mensagens saem em
     texto corrido.

O `cli_webots.py` também inicia o segundo controlador do mundo, o `grid_pose`, que dá a posição real do robô.
Sem ele, o Webots não avança a simulação.

Se o `WEBOTS_HOME` estiver errado, ele avisa `webots-controller not found: start grid_pose yourself, with
GRID_START="..."`. Nesse caso, inicie o `grid_pose` em outro terminal, na mesma pasta e com o mesmo
`WEBOTS_HOME`. Defina antes o `GRID_START` que a mensagem mostrou, por exemplo `"2 1 N"`:
`$env:GRID_START = "2 1 N"` no PowerShell, ou `export GRID_START="2 1 N"` no macOS e no Linux. Depois
troque o final do comando por:

```
--robot-name=grid_pose ../grid_pose/grid_pose.py
```

Sem o `GRID_START`, o robô fica onde o mundo o colocou, em (0, 0) olhando para o leste.

Antes de cada nova rodada: **⟲ (reset)** no Webots, **▶**, e o comando de novo.

## 3. Offline (sem Webots)

A FSM roda num mundo 2D simulado, com a câmera simulada. Bom para conferir que nada quebrou.

```bash
cd <repo>/Webots/Simulation_Webots/Simulation_Webots/controllers/fsm/offline
python scenarios.py                   # os 22 cenários (vários minutos)
python scenarios.py mission_blocked   # só um
python timeline.py ../fsm_log.csv     # linha do tempo de uma rodada do Webots (matplotlib)
```

No macOS e no Linux, use `python3`. Cada cenário imprime `PASS` ou `FAIL`, e os arquivos de cada rodada ficam
em `offline/results/` (não versionado).

## Problemas comuns

| Mensagem | O que fazer |
|---|---|
| `Set the path to your webots installation folder in WEBOTS_HOME` | Defina o `WEBOTS_HOME` no terminal (modo 2, passo 2) |
| PowerShell: `Token '...' inesperado` ou `O operador '--' funciona apenas em variáveis` | Falta o `&` antes do caminho entre aspas |
| Parado em `connecting to Webots ...` | Abra um mundo `*_extern.wbt`, aperte ⟲ e depois ▶ |
| `ModuleNotFoundError: No module named 'numpy'` (ou `'rich'`) | Instale o pacote no Python que o Webots usa (seção 0) |
| `pose lost at (x, y): saw [...], map has [...]` | O robô não estava onde a missão esperava (por exemplo, errou uma virada, ou a posição inicial informada não era a dele). Dê ⟲ antes de cada rodada e confira o nó e a direção que você informou ao abrir a CLI |
| `Texture image size ... is not a power of two` | Só um aviso do Webots, pode ignorar |

## Onde está cada coisa

| Pasta | O que tem |
|---|---|
| `SIM/worlds/` | os mundos; `make_grid_5x5.py` refaz a textura e os mundos da grade |
| `SIM/protos/` | o robô (`Cherokey_4W_v3`), a câmera e o sonar |
| `SIM/controllers/fsm/` | a FSM, os ganchos do Webots e o `cli_webots.py` (detalhes no `README.md` dessa pasta) |
| `SIM/controllers/grid_pose/` | supervisor: a posição real do robô na grade |
| `SIM/controllers/mission_view/` | supervisor opcional: desenha a missão com esferas (adicione um `Robot` com `supervisor TRUE` e `controller "mission_view"` ao mundo) |
| `SIM/controllers/basique/` | o primeiro controlador (pista em 8) |
