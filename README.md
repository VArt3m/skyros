
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
