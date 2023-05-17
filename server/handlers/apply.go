package handlers

import (
	"fmt"
	"io/ioutil"
	"log"
	"net/http"
	"os"
	"server/queries"
)

type DataBase struct {
	l *log.Logger
}

func NewDataBase(l *log.Logger) *DataBase {
	return &DataBase{l}
}

func (db *DataBase) ServeHTTP(rw http.ResponseWriter, r *http.Request) {
	q := queries.NewQuery("")

	q.FromJSON(r.Body)

	requestURL := "http://localhost:7200/repositories/Project?query=" + q.GetQuery()

	req, err := http.NewRequest(http.MethodGet, requestURL, nil)
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

	resBody, err := ioutil.ReadAll(res.Body)
	if err != nil {
		fmt.Printf("client: could not read response body: %s\n", err)
		os.Exit(1)
	}

	fmt.Printf("client: response body: %s\n", resBody)
}
