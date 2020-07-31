package main

import (
	"bufio"
	"fmt"
	"net"

	"github.com/lib/pq"
)

//User struct with fields username password
type User struct {
	username string
	password string
}

//SignUp ....
func SignUp(conn net.Conn) (User, error) {
	fmt.Fprintf(conn, "*****  WELCOME TO SignUp MENU  ******\n")
	input := bufio.NewScanner(conn)
	for {
		fmt.Fprintf(conn, "Enter username:")
		input.Scan()
		username := input.Text()
		fmt.Fprintf(conn, "Enter password:")
		input.Scan()
		password := input.Text()
		_, err := DB.Exec("INSERT INTO users (username, hash) VALUES ($1,$2)", username, password)
		if err != nil {
			fmt.Println(err)
			pqErr := err.(*pq.Error)
			if pqErr.Code == "23505" {
				fmt.Fprintf(conn, "Username Taken !!! \n")
				continue
			}
			return User{}, err
		}
		return User{username, password}, nil
	}

}

//Login ...
func Login(conn net.Conn) (User, error) {
	fmt.Fprintf(conn, "*****  WELCOME TO LOGIN MENU  ******\n")
	input := bufio.NewScanner(conn)
	for {
		fmt.Fprintf(conn, "Enter username:")
		input.Scan()
		name := input.Text()
		fmt.Fprintf(conn, "Enter password:")
		input.Scan()
		password := input.Text()
		row := DB.QueryRow("SELECT username from users WHERE  username=$1 AND hash = $2", name, password)
		var username string
		err := row.Scan(&username)
		if err != nil {
			fmt.Fprintf(conn, "Wrong Credentials try again or signup (t/s)")
			input.Scan()
			if input.Text() == "s" {
				return SignUp(conn)
			}
			continue
		}
		return User{username, password}, nil
	}

}
