package main

import (
	"log"
	"net/http"
)

type ConnectDB struct {
	l *log.Logger
}

func newConnection(l *log.Logger) *ConnectDB {
	return &ConnectDB{l}
}

func (con *ConnectDB) ServeHTTP(rw http.ResponseWriter, r *http.Request) {
	con.l.Printf("Start connecting.")

	con.l.Printf("Connected")

}
