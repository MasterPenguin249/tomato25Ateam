# day3-crane_ik

## Abstract
第3回　逆運動学を解きrvis上でcraneの手先を動かすパッケージです．

## Build
第2回でダウンロードしたリポジトリがホームディレクトリにあると思います．  
もし無い場合は，`day2-crane/README.md`を参考にしてダウンロードしてください．
```bash
$ cd ~/tomato2021
$ git pull
$ cp -r day3-crane_ik ~/catkin_ws/src/
$ cd ~/catkin_ws && catkin build
```

## Run
ビルドができたら，プログラムを実行します．  
端末を開き，以下を実行してください．
```bash
$ roslaunch day3-crane_ik arm4d_ik.launch
```
rvisが起動し，craneが2点間を交互に移動することを確認できれば大丈夫です．
