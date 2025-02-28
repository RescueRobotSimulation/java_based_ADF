# How to Use TaskManager System
16期 Rescue Simulation League リーダーの堀江孝文です！
動的タスク管理システムの創始者として使い方を説明していきたいとおもいます！

## これは何？


## どうやって使うの？
### Agent 側でしなきゃいけないこと
    - クラスメンバとしての登録
        _taskmanager モジュールを読み込み, マネージャーをメンバ変数として登録する
    - 初期化処理の登録
        各モジュールの resume, preparate で初期化処理する
        自分たちで作ったEvaluator を .setEvaluator(new クラス名()) で登録する
    - 毎回の実行
        各モジュールの calc or それに相当する関数の中で実行する

### Evaluator で実装しなきゃいけないの関数の中身
    - init()
        シミュレーションの一番最初に一回だけ
    - update()
        各サイクルで一回だけ
    - calc(EntityID target)
        taregt が渡された時にその優先度と不安定度を(RIOTaskKeyとして)返す
        各サイクルでループして行われる
    - その他正規化に関する関数も実装しなきゃいけない

### 関係するクラス


## リファレンス
### 実装の記録