package handlers

import (
	"context"
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

func (db *DataBase) SelectRequest(rw http.ResponseWriter, r *http.Request) {
	db.l.Printf("SELECT REQUEST")
	q := r.Context().Value(KeyQuery{}).(*queries.Query)

	// db.l.Println("query: " + q.GetQuery())

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
	rw.Write(resBody)
}

func (db *DataBase) UpdateDB(rw http.ResponseWriter, r *http.Request) {
	db.l.Printf("UPDATE REQUEST")
	q := r.Context().Value(KeyQuery{}).(*queries.Query)

	requestURL := "http://localhost:7200/repositories/Project/statements?update=" + q.GetQuery()
	req, err := http.NewRequest(http.MethodPost, requestURL, nil)

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
	rw.Write(resBody)
}

type KeyQuery struct{}

func (db *DataBase) MiddlewareDBValidation(next http.Handler) http.Handler {
	return http.HandlerFunc(func(rw http.ResponseWriter, r *http.Request) {
		q := &queries.Query{}

		q.FromJSON(r.Body)

		ctx := context.WithValue(r.Context(), KeyQuery{}, q)
		request := r.WithContext(ctx)

		next.ServeHTTP(rw, request)
	})
}
