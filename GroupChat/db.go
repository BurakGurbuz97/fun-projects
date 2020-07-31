package main

import (
	"database/sql"
	"fmt"
	"log"
)

//Connection ...
func Connection() *sql.DB {
	db, err := sql.Open("postgres", "postgres://postgres:*********@127.0.0.1:5432/ChatApp?sslmode=disable")

	if err != nil {
		log.Fatal("CANNOT CONNECT TO DATABASE")
	}
	fmt.Println("SUCCESSFULLY CONNECTED TO DATABASE")
	return db
}

//DB connection
var DB = Connection()
