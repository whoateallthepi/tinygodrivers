.DEFAULT_GOAL := build

fmt:
	go fmt ./...
.PHONY:fmt

lint: fmt
	golint ./...
.PHONY:lint

#vet: fmt
#	go vet ./...
#.PHONY:vet

build: lint #vet
	tinygo build -target=pico -o bin/main.elf main.go
