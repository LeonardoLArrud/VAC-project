# Simulação no Webots

O robô segue a linha, detecta as interseções, para diante de obstáculos e navega na grade 5x5 com Dijkstra.
A navegação (`navigators.py` e `Robo_FSM/graph/`) é a mesma do robô real.

`SIM` = `Webots/Simulation_Webots/Simulation_Webots`.

## Instalação

- **Webots R2025a**: <https://cyberbotics.com>.
- **Python 3** (testado com o 3.11), com `numpy` e `rich` (e `opencv-python` e `matplotlib` para o teste offline):

  | SO | Comando |
  |---|---|
  | Windows | `python -m pip install numpy rich opencv-python matplotlib` |
  | macOS | `python3 -m pip install numpy rich opencv-python matplotlib` |
  | Ubuntu/Debian | `sudo apt install python3-numpy python3-rich python3-opencv python3-matplotlib` |

- O Webots usa o `python` (Windows) ou o `python3` (macOS e Linux) do PATH. Para usar outro, como o de um `venv`,
  mude em **Tools → Preferences → General → Python command**.

## 1. Só o Webots

1. *(Opcional)* Para uma missão, edite `SIM/controllers/fsm/fsm.py`:
   `MISSION = ("MISSION", (0, 0), "E", (4, 3))` (nó inicial, direção, destino).
   x cresce para a direita, y para cima, e (0, 0) é o canto de baixo à esquerda.
2. **File → Open World →** `SIM/worlds/grid_5x5.wbt` e **▶**. O robô parte sozinho após 1 s.

- **Teclas** (com a vista 3D clicada): `G` começa, `S` para, `M` reenvia a missão, `L` tampa a câmera.
- **Câmera:** <http://localhost:8000/> mostra a câmera, e <http://localhost:8000/mask> o que o seguidor enxerga.
- **Outros mundos:**
  - `grid_5x5_box.wbt`: uma caixa entre (2, 3) e (3, 3); com a missão acima, o robô volta e recalcula a rota;
  - `grid_5x5_reflexos.wbt`: reflexos no chão;
  - `huit.wbt`: o 8; mude o `controller` do robô para `fsm`, senão roda o controlador antigo.

## 2. Entregas digitadas no terminal

1. Abra `SIM/worlds/grid_5x5_extern.wbt` (ou `grid_5x5_box_extern.wbt`) e aperte **▶**.
2. Num terminal:

   **Windows (PowerShell)**
   ```powershell
   $env:WEBOTS_HOME = "C:\Program Files\Webots"    # ou "$env:LOCALAPPDATA\Programs\Webots"
   cd <repo>\Webots\Simulation_Webots\Simulation_Webots\controllers\fsm
   & "$env:WEBOTS_HOME\msys64\mingw64\bin\webots-controller.exe" --robot-name=Cherokey_4W_v3 cli_webots.py --lp
   ```

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
3. Informe o nó inicial e a direção (Enter: `0 0` e `E`), ou passe `--start 2 1 --heading N` no fim do comando.
4. Digite um destino `x y` (de 0 a 4) e Enter. Durante a viagem, `q` (sem Enter) para o robô. Na chegada,
   digite o próximo destino, ou `q` + Enter para sair.

**Voltas no 8:** abra `huit_extern.wbt` e troque `--lp` por `--eight`. Digite o número de voltas (`0` = até `q`).
No Webots, o cruzamento do 8 não é detectado, então as voltas não são contadas: pare com `q`.

Antes de cada rodada: **⟲** e **▶**.

## 3. Offline (sem Webots)

```bash
cd <repo>/Webots/Simulation_Webots/Simulation_Webots/controllers/fsm/offline
python scenarios.py                   # os 22 cenários, PASS ou FAIL (demora alguns minutos)
python scenarios.py mission_blocked   # um só
```

## Problemas comuns

| Mensagem | Solução |
|---|---|
| `Set the path to your webots installation folder in WEBOTS_HOME` | Defina o `WEBOTS_HOME` (seção 2, passo 2) |
| `webots-controller not found: start grid_pose yourself` | O `WEBOTS_HOME` aponta para a pasta errada |
| PowerShell: `O operador '--' funciona apenas em variáveis` | Falta o `&` antes do caminho entre aspas |
| Parado em `connecting to Webots` | Abra um mundo `*_extern.wbt` e aperte ⟲ e ▶ |
| `No module named 'numpy'` (ou `'rich'`) | Instale o pacote no Python que o Webots usa |
| `pose lost at (x, y)` | O robô não estava onde a missão supunha: ⟲, ▶ e confira o nó e a direção informados |
| `Texture image size ... is not a power of two` | Aviso do Webots, pode ignorar |
