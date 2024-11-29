
# Skyros


## Installation

Install uv
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
```

Install Skyros
```bash
git clone https://github.com/VArt3m/skyros.git
cd skyros
uv sync
```

Update Skyros
```bash
git pull --rebase --autostash
uv sync
```


## Usage

Perf tests
```bash
uv run tests/perf/recv.py

uv run tests/perf/send.py --peer tcp/51.250.24.185:7447 --packet-size 128
```
