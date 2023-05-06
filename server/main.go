package main

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"time"
)

func sendTTL(file_path string) {
	kvPairs := make(map[string]string)
	kvPairs["config"] = "@" + file_path
	postJson, err := json.Marshal(kvPairs)
	if err != nil {
		panic(err)
	}

	postContent := bytes.NewBuffer(postJson)

	request, err := http.Post("localhost:7200/rest/repositories", "multipart/form-data", postContent)
	if err != nil {
		log.Panic("Wrong api")
	}
	fmt.Println(request)
}

func main() {
	l := log.New(os.Stdout, "db-api", log.LstdFlags)
	ch := newConnection(l)

	sm := http.NewServeMux()
	sm.Handle("/", ch)

	s := &http.Server{
		Addr:         ":9090",
		Handler:      sm,
		IdleTimeout:  120 * time.Second,
		ReadTimeout:  20 * time.Second,
		WriteTimeout: 20 * time.Second,
	}

	go func() {
		err := s.ListenAndServe()
		if err != nil {
			l.Fatal(err)
		}
	}()
	sendTTL("atlas_8_floor.ttl")

	sigChan := make(chan os.Signal)
	signal.Notify(sigChan, os.Interrupt)
	signal.Notify(sigChan, os.Kill)

	sig := <-sigChan
	l.Println("Received terminate, graceful shutdown", sig)

	tc, _ := context.WithTimeout(context.Background(), 30*time.Second)
	s.Shutdown(tc)
}
