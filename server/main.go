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

	// go func() {
	// 	requestURL := fmt.Sprintf("http://localhost:7200/rest/repositories/Project/")
	// 	req, err := http.NewRequest(http.MethodGet, requestURL, nil)
	// 	if err != nil {
	// 		fmt.Printf("client: could not create request: %s\n", err)
	// 		os.Exit(1)
	// 	}

	// 	res, err := http.DefaultClient.Do(req)
	// 	if err != nil {
	// 		fmt.Printf("client: error making http request: %s\n", err)
	// 		os.Exit(1)
	// 	}

	// 	fmt.Printf("client: got response!\n")
	// 	fmt.Printf("client: status code: %d\n", res.StatusCode)

	// 	resBody, err := ioutil.ReadAll(res.Body)
	// 	if err != nil {
	// 		fmt.Printf("client: could not read response body: %s\n", err)
	// 		os.Exit(1)
	// 	}
	// 	fmt.Printf("client: response body: %s\n", resBody)
	// }()

	go func() {
		jsonBody := []byte(`{"element": "Columns"}`)
		bodyReader := bytes.NewReader(jsonBody)

		requestURL := fmt.Sprintf("http://localhost:7200")
		req, err := http.NewRequest(http.MethodPost, requestURL, bodyReader)

		if err != nil {
			fmt.Printf("client: could not create request: %s\n", err)
			os.Exit(1)
		}

		res, err := http.DefaultClient.Do(req)
		if err != nil {
			fmt.Printf("client: error making http request: %s\n", err)
			os.Exit(1)
		}

		fmt.Printf("client: got response!\n")
		fmt.Printf("client: status code: %d\n", res.StatusCode)

		// resBody, err := ioutil.ReadAll(res.Body)
		// if err != nil {
		// 	fmt.Printf("client: could not read response body: %s\n", err)
		// 	os.Exit(1)
		// }
		fmt.Print("client: response body: ", res)
	}()

	sigChan := make(chan os.Signal)
	signal.Notify(sigChan, os.Interrupt)
	signal.Notify(sigChan, os.Kill)

	sig := <-sigChan
	l.Println("Received terminate, graceful shutdown", sig)

	tc, _ := context.WithTimeout(context.Background(), 30*time.Second)
	s.Shutdown(tc)
}
