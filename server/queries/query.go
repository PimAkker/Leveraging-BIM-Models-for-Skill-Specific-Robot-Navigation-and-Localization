package queries

import (
	"encoding/json"
	"io"
	"log"
	"strconv"
	"strings"
)

type Query struct {
	query string `validate:"required"`
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
	for i := 0; i < len(q.query); i++ {
		if !is_letter(q.query[i]) && q.query[i] != '%' && q.query[i] != '.' {
			mask := strconv.FormatInt(int64(q.query[i]), 16)
			q.query = strings.ReplaceAll(q.query, string(q.query[i]), "%"+mask)
		}
	}
}

func (q Query) GetQuery() string {
	return q.query
}
