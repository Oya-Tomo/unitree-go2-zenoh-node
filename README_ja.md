# Unitree Go2 Zenoh Node

[English](README.md)

Zenoh commandを最新のUnitree Go2 `SportModeState`と照合してから
`SportClient`へ渡す、単独実行nodeです。

> [!WARNING]
> このソフトウェアは実機を動かします。機体を支持し、無線remoteと緊急停止手順を
> 用意し、最初は低速で試してください。

## アーキテクチャ

```text
DDS SportModeState ---> 最新State inbox ---\
                                         +--> State駆動loop --> SDK
Zenoh command -------> 最新command buffer /
                              |
                              +--> {robot_key}/state
```

loopの実行順序は常に同じです。

1. 未処理の最新実機Stateを読む
2. 最新の姿勢・速度bufferを確認する
3. 姿勢を優先して条件分岐する
4. 最大1回だけSDKを呼ぶ

Zenoh callbackはSDKを呼びません。中心となる実装は、processとI/Oを構成する
`node.py`、Stateと制御方針を持つ`controller.py`、設定だけを持つ`config.py`の
3ファイルです。詳細は
[ADR-0001](docs/adr/0001-observed-state-driven-control-architecture.md)にあります。

## インストール

Python 3.13、uv、ローカルbuildしたCycloneDDS、到達可能なGo2が必要です。

```bash
git clone --recurse-submodules https://github.com/Oya-Tomo/unitree-go2-zenoh-node.git
cd unitree-go2-zenoh-node
git submodule update --init --recursive

export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
uv sync --all-groups
```

## 設定

```bash
cp config/node-config.example.json5 config/node-config.json5
cp config/zenoh-config.example.json5 config/zenoh-config.json5
cp examples/keyboard-config.example.json5 examples/keyboard-config.json5
cp examples/keyboard-zenoh-config.example.json5 examples/keyboard-zenoh-config.json5
```

`dds.network_interface`をGo2に接続したinterfaceへ変更します。nodeとkeyboardでは
同じ具体的な`robot_key`を使います。`maximum_age_seconds`はStateを信頼できる時間、
速度閾値は実測値をmovingまたはquiescentに分けるための値です。

Zenoh exampleには認証・暗号化がないため、信頼できるLANだけで使ってください。

## 実行

先にrobot nodeを起動します。

```bash
export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
uv run node.py \
  --node-config config/node-config.json5 \
  --zenoh-config config/zenoh-config.json5
```

次にkeyboard clientを起動します。

```bash
uv run --group example python -m examples.keyboard \
  --keyboard-config examples/keyboard-config.json5 \
  --zenoh-config examples/keyboard-zenoh-config.json5
```

## Command仕様

`{robot_key}/command`へJSONをpublishします。

```json
{"type":"velocity","vx":0.5,"vy":0.0,"vyaw":0.2}
```

```json
{"type":"posture","posture":"stand"}
```

```json
{"type":"posture","posture":"down"}
```

速度と姿勢はそれぞれ最新値だけを保持し、姿勢を速度より優先します。明示的なstop
commandはありません。zero速度には`Move(0, 0, 0)`を使います。

## State駆動の動作

freshな`SportModeState` 1 sampleを即座に採用します。SDK return valueは診断情報であり、
姿勢やmotionを確定する根拠にはしません。

| 実機State | Stand request | Down request | Velocity |
| --- | --- | --- | --- |
| Quiescent Down | `StandUp` | 完了 | 待機 |
| Idle stand | `BalanceStand` | Stop後に`StandDown` | 待機 |
| Ready stand | 完了 | Stop後に`StandDown` | `Move` |
| Locomotion | `BalanceStand` | Stop後に`StandDown` | `Move` |
| Transition、moving mode 5、unsupported、fault、Unknown | 待機 | 待機 | 待機 |

姿勢に関係するSDK callの直前に、公開する実機Stateを`Unknown`にします。RPC実行中も
Unknownのままです。新しいvalid Stateを受信するまでは、届いたcommandをすべて
無視します。

Downは次の固定workflowです。

```text
non-down State -> StopMoveを1回 -> 新しいquiescent State
               -> StandDown -> 新しいdown State -> 完了
```

`StopMove()`は起動時、Stand、zero速度では使いません。`-1`でも再試行しません。
次のStateがmovingのままなら、2回目の`StopMove()`も`StandDown()`も送らず待機します。

終了時も同じStop→Downを行います。quiescent downなら、どちらも送りません。
mode 5でも実測速度がmovingならtransitionとして扱い、Stand、Down、終了を完了
しません。

## 公開State

nodeは`{robot_key}/state`へpublishし、`get`にも応答します。実機由来State、request、
最新SDK診断を分けて表示し、command名を物理姿勢として扱いません。

## Keyboard操作

Shiftは速度deadmanであり、姿勢requestにも必要です。

| Input | Action |
| --- | --- |
| `Shift+W/S` | `vx`正 / 負 |
| `Shift+A/D` | `vy`正 / 負 |
| `Shift+Q/E` | `vyaw`正 / 負 |
| `Shift+R` | Standを1回publish |
| `Shift+F` | Downを1回publish |
| `Space` | zero速度をpublish |
| Shift解放 / focus喪失 | zero速度をpublish |
| `Esc` / close | zero速度をpublishして終了 |

## 検証

robot nodeを起動せずに実行できます。

```bash
uv run ruff format --check .
uv run ruff check .
uv run pyright
uv run python -m unittest discover -s tests -v
```

実機acceptanceでは、伏せ起動、Stand、低速歩行、Down、再Stand、終了を、full Stateと
SDK診断を記録しながら確認してください。
