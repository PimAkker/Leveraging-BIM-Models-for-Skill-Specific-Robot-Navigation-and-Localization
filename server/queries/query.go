package queries

import (
	"encoding/json"
	"io"
	"log"
	"net/url"
	"strings"
)

type Query struct {
	Query string `json:"query"`
}

func NewQuery(query string) *Query {
	return &Query{query}
}

func (q *Query) FromJSON(r io.Reader) {
	e := json.NewDecoder(r)
	err := e.Decode(q)
	if err != nil {
		log.Fatal("error when converting from json", r)
	}
	q.translateQuery()
}

func is_letter(ch byte) bool {
	return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')
}

func (q *Query) translateQuery() {
	q.Query = url.QueryEscape(q.Query)
	q.Query = strings.ReplaceAll(q.Query, "+", "%20")
	q.Query = strings.ReplaceAll(q.Query, "%22", "'")
}

func (q Query) GetQuery() string {
	return q.Query
}
