# 仮想環境の作成方法

## pyenv

`pyenv`をインストールする．

```shell
cd ~
git clone https://github.com/pyenv/pyenv.git .pyenv
# install build deps
sudo apt update; sudo apt install -y build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev curl \
libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev
# write settings into .bashrc
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init -)"' >> ~/.bashrc
```

別シェルを立ち上げて`pyenv-virtualenv`をインストールする．

```shell
git clone https://github.com/pyenv/pyenv-virtualenv.git $(pyenv root)/plugins/pyenv-virtualenv
echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc
```

pythonのインストールと仮想環境の作成を行う．

下の例ではpython3.10.14がインストールされ`pyHailoRT`という名前の仮想環境が作成される．

```shell
pyenv install 3.10.14
pyenv virtualenv 3.10.14 pyHailoRT
```

ディレクトリを作成し仮想環境を適用する．

```shell
mkdir playground
cd playground
pyenv local pyHailoRT
```

## venv

`venv`をインストールする．

```shell
$ sudo apt install python3.12-venv
```

仮想環境を作成する．

```shell
python3.12 -m venv venv
```
仮想環境をアクティベートする．

```shell
source POV_Demo_Device/work/venv/bin/activate
```