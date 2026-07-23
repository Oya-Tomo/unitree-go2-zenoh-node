# セットアップ

## 必要なもの

- 専用の有線network interfaceから到達できるUnitree Go2 Edu
- V1.1.6以降のGo2 software
- Python 3.13、Git、[uv](https://docs.astral.sh/uv/)を利用できるLinux PC
- 同梱SDKと互換性のあるローカルCycloneDDS installation
- pygame keyboard controllerを表示するdesktop環境
- Go2の無線remoteと確認済みの緊急停止手順

このnodeはUnitreeのhigh-level SportClient APIを使います。同じhigh-level serviceを別processが操作している状態では起動しないでください。

## インストール

SDK submoduleと一緒にrepositoryをcloneし、repository rootへ移動します。

```console
$ git clone --recurse-submodules https://github.com/Oya-Tomo/unitree-go2-zenoh-node.git
$ cd unitree-go2-zenoh-node
$ git submodule update --init --recursive
```

CycloneDDS Python buildへローカルCycloneDDSの場所を指定し、nodeとkeyboardの依存関係をインストールします。

```console
$ export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
$ uv sync --all-groups
```

このPCで実際に使っているinstall prefixを指定してください。Unitree SDKを使うcommandを実行するときも、shell environmentに`CYCLONEDDS_HOME`を残します。依存関係のinstall時にCycloneDDSが見つからないと表示された場合は、prefix内にinstall済みlibraryとCMake metadataがあることを確認してから`uv sync`を再実行してください。

既定の設定pathはcurrent working directoryからの相対pathです。すべての設定pathを明示しない場合はrepository rootからcommandを実行してください。

## 設定ファイルの準備

管理対象のexampleを、Gitから除外される実行時pathへコピーします。

```console
$ cp config/node-config.example.json5 config/node-config.json5
$ cp config/zenoh-config.example.json5 config/zenoh-config.json5
$ cp examples/keyboard-config.example.json5 examples/keyboard-config.json5
$ cp examples/keyboard-zenoh-config.example.json5 examples/keyboard-zenoh-config.json5
```

| 実行時ファイル | 用途 |
| --- | --- |
| `config/node-config.json5` | Go2 DDS interface、State watchdog、制御timeout、`robot_key` |
| `config/zenoh-config.json5` | robot nodeのZenoh mode、listener、接続、scouting |
| `examples/keyboard-config.json5` | keyboardの目標速度、ramp rate、表示freshness、`robot_key` |
| `examples/keyboard-zenoh-config.json5` | keyboardのZenoh接続 |

これらはPC・network固有の値を含むため、実行時ファイルはGitの管理対象外です。nodeとkeyboardのapplication設定はstrictなJSON5であり、必須fieldの欠落、未知のfield、誤った型は起動時errorになります。Zenoh設定はZenoh libraryが検証します。

## Go2用DDS interfaceの選択

PC上のinterfaceを確認します。

```console
$ ip -br link
$ ip -br address
```

Go2へ有線接続したinterfaceを特定し、upになっていることを確認します。interface名が`enp3s0`の場合は、次のcommandが確認に使えます。

```console
$ ip link show enp3s0
$ ip address show dev enp3s0
$ ip neigh show dev enp3s0
```

IP addressではなくinterface名を`config/node-config.json5`へ設定します。

```json5
dds: {
  domain_id: 0,
  network_interface: "enp3s0",
  rpc_timeout_seconds: 0.2,
  sport_mode_state_topic: "rt/sportmodestate",
}
```

exampleのinterface名はplaceholderです。このPCで確認せずにコピーしないでください。DDS trafficはこのinterfaceを使い、robot nodeとkeyboardのZenoh接続とは独立しています。

## Zenohでnodeとkeyboardを接続する

`config/node-config.json5`と`examples/keyboard-config.json5`では、同じconcreteなZenoh keyを`robot_key`に設定します。同梱値は次のとおりです。

```json5
robot_key: "unitree/go2"
```

2つのZenohファイルはprocess同士の接続方法を決めます。DDS interfaceを選ぶ設定ではありません。

### 両processを1台のPCで動かす場合

両方で同じTCP portをbindせず、片方をlistener、もう片方をclientにします。loopbackだけでlistenするrobot node設定は次のとおりです。

```json5
{
  mode: "peer",
  listen: {
    endpoints: ["tcp/127.0.0.1:7447"],
  },
  connect: {
    endpoints: [],
  },
  scouting: {
    multicast: {
      enabled: false,
    },
  },
}
```

keyboardはこのlistenerへ接続するclientにします。

```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/127.0.0.1:7447"],
  },
  scouting: {
    multicast: {
      enabled: false,
    },
  },
}
```

### 別hostで動かす場合

robot nodeを到達可能なinterfaceでlistenさせ、keyboardからそのhostのaddressへ接続します。たとえばnodeが`tcp/0.0.0.0:7447`でlistenする場合、keyboardは`tcp/192.168.1.20:7447`のようにnode PCの到達可能なaddressへ接続します。

`0.0.0.0`はローカルのwildcard bind addressであり、remote側の`connect.endpoints`へ書くaddressではありません。port 7447を別listenerが使っていない必要があります。全interfaceでlistenすると、到達可能な全networkへendpointを公開するため、必要に応じてbind addressやfirewallを制限してください。

example Zenoh設定には認証や暗号化がありません。信頼できるnetworkだけで使ってください。別のtopologyを選ぶ場合は[Zenoh deployment guide](https://zenoh.io/docs/getting-started/deployment/)を参照してください。

## 別の場所にある設定を使う

robot nodeではnode設定とZenoh設定のpathを明示できます。

```console
$ uv run node.py \
    --node-config /etc/unitree-go2-zenoh-node/node.json5 \
    --zenoh-config /etc/unitree-go2-zenoh-node/zenoh.json5
```

keyboardではkeyboard設定とZenoh設定のpathを明示できます。

```console
$ uv run examples/keyboard.py \
    --keyboard-config /etc/unitree-go2-zenoh-node/keyboard.json5 \
    --zenoh-config /etc/unitree-go2-zenoh-node/keyboard-zenoh.json5
```

個々の設定値をCLIから上書きすることはできません。

## 安全確認

どちらのprocessも起動する前に、次を確認します。

1. Go2を障害物のない水平な場所へ置き、最初の姿勢変更中は機体を支持する。
2. 人、cable、障害物をrobotの可動範囲外へ移動する。
3. 無線remoteを手元に置き、緊急停止手順を確認する。
4. 最初はkeyboard exampleの保守的な目標速度を使う。
5. 専用DDS interfaceと、両application設定の`robot_key`を確認する。
6. robot nodeを先に起動し、freshなStateを確認してからkeyboardを起動する。

Zenoh keyboardやprocess終了操作は緊急停止の代わりにはなりません。

## 開発時の検証

次のcheckはrobot nodeを起動せず、Go2へcommandを送りません。

```console
$ uv run ruff format --check .
$ uv run ruff check .
$ uv run pyright
$ uv run python -m unittest discover -s tests -v
```

setup完了後は[使い方](usage.md)へ進んでください。
