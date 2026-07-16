# Unitree Go2 Zenoh Node

[English](README.md)

Zenoh経由で速度・姿勢commandを受信し、high-level SportClient APIを使って1台のUnitree Go2を制御する単独実行nodeです。

> [!WARNING]
> このソフトウェアは実機を動かします。機体を支持し、無線remoteと緊急停止手順を用意し、最初は低速で試してください。

## クイックスタート

submoduleを初期化済みのcheckoutで実行します。

```console
$ export CYCLONEDDS_HOME="$HOME/Packages/cyclonedds/install"
$ uv sync --all-groups
$ cp config/node-config.example.json5 config/node-config.json5
$ cp config/zenoh-config.example.json5 config/zenoh-config.json5
$ cp examples/keyboard-config.example.json5 examples/keyboard-config.json5
$ cp examples/keyboard-zenoh-config.example.json5 examples/keyboard-zenoh-config.json5
```

nodeを起動する前に、このPCとnetworkに合わせて4つの実行時設定を編集してください。DDS・Zenohを含む手順は[セットアップ](docs/ja/setup.md)にあります。

robot nodeを起動してからkeyboard controllerを起動します。

```console
$ uv run node.py
$ uv run --group example python -m examples.keyboard
```

実機へcommandを送る前に[使い方](docs/ja/usage.md)を確認してください。

## ドキュメント

- [セットアップ](docs/ja/setup.md)
- [使い方・操作・wire contract](docs/ja/usage.md)
- [アーキテクチャ決定記録](docs/adr/0001-observed-state-driven-control-architecture.md)
- [English documentation](README.md)
