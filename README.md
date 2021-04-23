# cloud9

### cloud9に移植する用

## 気をつけること
robot_wsをまるごと交換してもうまくいきません。

aws_game_managerや、delivery_robot_sampleに初期セットアップ時に取得した個人の情報が含まれているからです。

なので、そういうことを踏まえて、うまく移行作業をする必要があります。
## How to checkout branch
```
git checkout --recurse-submodules cloud9
```