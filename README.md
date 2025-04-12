# コンテナ（実験環境）の起動方法
##　前提としてdockerをダウンロードして下さい

## Linuxの場合

1. 好きなディレクトリで以下のコードを実行してください

    ```bash
    git clone https://github.com/tomsan603/ros2_experiment.git　
    ```

2. 実行権限を付与します(初回のみ)

    ```bash
    cd ./ros2_experiment
    chmod +x ./run.bash
    ```

3. コンテナを起動します

    ```bash
    ./run.bash
    ```

4. コンテナを停止する場合はコンテナ内のbashをログアウトします

    ```bash
    exit
    ```
5. 実行権限を付与します(初回のみ)

    ```bash
    chmod +x ./stop.bash
    ```

6. コンテナを停止します

    ```bash
    ./stop.bash
    ```

## Windowsの場合

1. 好きなディレクトリで以下のコードを実行してください

    ```bash
    git clone https://github.com/tomsan603/ros2_experiment.git　
    ```

2. ディレクトリを移動します

    ```bash
    cd ./ros2_experiment
    ```

3. コンテナを起動します

    ```bash
    ./run.bat
    ```

4. コンテナを停止する場合はコンテナ内のbashをログアウトします

    ```bash
    exit
    ```

5. コンテナを停止します

    ```bash
    ./stop.bat
    ```
# コンテナ内での操作方法
# 実験方法

