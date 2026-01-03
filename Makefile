.PHONY: help build run run-mapping run-shell stop clean

help:
	@echo "HomeCleanerBot Docker Commands"
	@echo "================================"
	@echo "make build         - Build Docker image"
	@echo "make run           - Run navigation mode"
	@echo "make run-mapping   - Run mapping mode"
	@echo "make run-shell     - Open interactive shell"
	@echo "make stop          - Stop all containers"
	@echo "make clean         - Remove containers and images"

build:
	@./build_docker.sh

run:
	@./run_docker.sh navigation

run-mapping:
	@./run_docker.sh mapping

run-shell:
	@./run_docker.sh bash

stop:
	@docker stop $$(docker ps -q --filter name=homecleanerbot) 2>/dev/null || true

clean:
	@docker rm -f $$(docker ps -aq --filter name=homecleanerbot) 2>/dev/null || true
	@docker rmi homecleanerbot:latest 2>/dev/null || true
