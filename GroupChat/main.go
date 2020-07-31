package main

import (
	"bufio"
	"fmt"
	"log"
	"net"
)

//INIT GROUPS READ FROM DATABASE

type client chan<- string

//GROUPS all created groups
var GROUPS = make(map[*Group]bool)

func handleConn(conn net.Conn) {
	input := bufio.NewScanner(conn)
	var user User
	for {
		fmt.Fprintf(conn, "Login or Signup (l/s): ")
		input.Scan()
		if input.Text() == "l" {
			u, err := Login(conn)
			user = u
			if err != nil {
				fmt.Fprintf(conn, "Unable to login\n")
				continue
			}
			break
		}
		if input.Text() == "s" {
			u, err := SignUp(conn)
			user = u
			if err != nil {
				fmt.Fprintf(conn, "Unable to login\n")
				continue
			}
			break
		}
	}
	GroupWelcome(conn, user)
}

func main() {
	listener, err := net.Listen("tcp", "localhost:8000")
	if err != nil {
		log.Fatal(err)
	}
	initGroups()
	for {
		conn, err := listener.Accept()
		if err != nil {
			log.Print(err)
			continue
		}
		go handleConn(conn)
	}
}
