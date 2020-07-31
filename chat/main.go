package main

import (
	"bufio"
	"database/sql"
	"fmt"
	"log"
	"net"
	"time"

	"github.com/lib/pq"
)

//Connection ...
func Connection() *sql.DB {
	db, err := sql.Open("postgres", "postgres://postgres:pass@127.0.0.1:5432/ChatApp?sslmode=disable")
	if err != nil {
		log.Fatal("CANNOT CONNECT TO DATABASE")
	}
	fmt.Println("Succesfully connected to database")
	return db
}

//Dbcon ...
var Dbconn = Connection()

type client chan<- string // an outgoing message channel

var (
	entering = make(chan client)
	leaving  = make(chan client)
	messages = make(chan string) // all incoming client messages
)

func broadcaster() {
	clients := make(map[client]bool) // all connected clients
	for {
		select {
		case msg := <-messages:
			//Broadcast incoming message to all
			//clients outgoing message channels
			for cli := range clients {
				cli <- msg
			}
		case cli := <-entering:
			clients[cli] = true
		case cli := <-leaving:
			delete(clients, cli)
			close(cli)
		}
	}
}

func clientWriter(conn net.Conn, ch <-chan string) {
	for msg := range ch {
		fmt.Fprintln(conn, msg)
	}
}

//Authentication ...
func Authentication(conn net.Conn) string {
	fmt.Fprintf(conn, "*****  WELCOME TO LOGIN MENU  ******\n")
	input := bufio.NewScanner(conn)
	for {
		fmt.Fprintf(conn, "Enter username:")
		input.Scan()
		name := input.Text()
		fmt.Fprintf(conn, "Enter password:")
		input.Scan()
		password := input.Text()
		row := Dbconn.QueryRow("SELECT username from users where username = $1 AND hash = $2 ", name, password)
		var username string
		err := row.Scan(&username)
		if err != nil {
			fmt.Fprintf(conn, "Invalid credentials try again or sign up (t/s)")
			input.Scan()
			flag := input.Text()
			if flag == "s" {
				return Signup(conn)
			}
			continue
		}
		return username
	}

}

//Signup ...
func Signup(conn net.Conn) string {
	fmt.Fprintf(conn, "*****  WELCOME TO SIGN UP MENU  ******\n")
	input := bufio.NewScanner(conn)
	for {
		fmt.Fprintf(conn, "Enter username:")
		input.Scan()
		username := input.Text()
		fmt.Fprintf(conn, "Enter password:")
		input.Scan()
		password := input.Text()

		_, err := Dbconn.Exec("INSERT INTO users (username, hash) VALUES ($1,$2)", username, password)
		if err != nil {
			fmt.Println(err)
			pqErr := err.(*pq.Error)
			if pqErr.Code == "23505" {
				fmt.Fprintf(conn, "USERNAME TAKEN !!!! \n")
				continue
			}
			return "error"
		}
		return username
	}
}

//Welcome ...
func Welcome(conn net.Conn) string {
	fmt.Fprintf(conn, "\rLogin or Signup (l/s): ")
	input := bufio.NewScanner(conn)
	input.Scan()
	text := input.Text()
	if text == "s" {
		return Signup(conn)
	}
	return Authentication(conn)

}

func getRecent(conn net.Conn) {
	rows, err := Dbconn.Query("SELECT msg from messages ORDER BY time DESC LIMIT 20")
	if err != nil {
		fmt.Fprintf(conn, "Unable to fethed past messages \n")
		fmt.Println(err)
		return
	}
	defer rows.Close()
	for rows.Next() {
		var m string
		rows.Scan(&m)
		fmt.Fprintf(conn, m+"\n")
	}
}

//Welcome

func handleConn(conn net.Conn) {
	ch := make(chan string) //outgoing client messages
	go clientWriter(conn, ch)
	who := Welcome(conn)
	if who == "error" {
		fmt.Fprintf(conn, "FATAL ERROR")
		conn.Close()
		return
	}
	getRecent(conn)
	input := bufio.NewScanner(conn)
	ch <- "You are " + who
	messages <- who + " has arrived"
	entering <- ch
	for input.Scan() {
		t := time.Now().Unix()
		msg := who + ": " + input.Text()
		_, err := Dbconn.Exec("INSERT INTO messages (sender, msg, time) VALUES ($1,$2,$3)", who, msg, t)
		if err != nil {
			fmt.Println(err)
			fmt.Fprintf(conn, "Unable to send message ... \n")
			continue
		}
		messages <- msg
	}

	leaving <- ch
	messages <- "\r" + who + " has left"
	conn.Close()
}

func main() {
	listener, err := net.Listen("tcp", "localhost:8000")
	if err != nil {
		log.Fatal(err)
	}
	go broadcaster()
	for {
		conn, err := listener.Accept()
		if err != nil {
			log.Print(err)
			continue
		}
		go handleConn(conn)
	}
}
