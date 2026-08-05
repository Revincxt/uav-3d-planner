.PHONY: install lint format typecheck test check build web-check web-build

install:
	python -m pip install -e '.[dev]'

lint:
	ruff check .
	ruff format --check .

format:
	ruff check --fix .
	ruff format .

typecheck:
	mypy src

test:
	pytest --cov=uav3d --cov-report=term-missing

web-check:
	cd web && pnpm validate:data && pnpm test && pnpm typecheck

web-build:
	cd web && pnpm build

check: lint typecheck test web-check

build:
	python -m build

