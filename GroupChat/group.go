package main

import (
	"bufio"
	"fmt"
	"log"
	"net"
	"time"
)

//Group struct represents chat group
type Group struct {
	Name     string
	Clients  map[client]bool
	Entering chan client
	Leaving  chan client
	Messages chan string
}

//NewGroup Group constructor
func NewGroup(name string) Group {
	g := Group{
		name,
		make(map[client]bool),
		make(chan client),
		make(chan client),
		make(chan string),
	}
	go g.groupBroadcaster()
	return g
}

//Listens connections and acts
func (group *Group) groupBroadcaster() {
	for {
		select {
		case msg := <-group.Messages:
			for cli := range group.Clients {
				cli <- msg
			}
		case cli := <-group.Entering:
			group.Clients[cli] = true
		case cli := <-group.Leaving:
			delete(group.Clients, cli)
			close(cli)
		}
	}
}

func initGroups() {
	rows, err := DB.Query("SELECT groupname from groups")
	defer rows.Close()
	if err != nil {
		log.Fatal("Unable to init groups")
	}
	for rows.Next() {
		var groupname string
		rows.Scan(&groupname)
		g := NewGroup(groupname)
		GROUPS[&g] = true
	}
	fmt.Println("Groups Succesfully Initialized")
}

//GroupWelcome ...
func GroupWelcome(conn net.Conn, user User) {
	ch := make(chan string) //outgoing client messages
	var MatchedGroup Group
INIT:
	for {
		fmt.Fprintf(conn, "Join or Create Group (j/c):")
		input := bufio.NewScanner(conn)
		input.Scan()
		if input.Text() == "j" {
			fmt.Fprintf(conn, "Enter Group Name:")
			input.Scan()
			groupName := input.Text()
			for g := range GROUPS {
				if g.Name == groupName {
					go g.groupBroadcaster()
					g.Entering <- ch
					g.Messages <- user.username + " has arrived "
					MatchedGroup = *g
					fmt.Println(user.username, " has joined to group ", g.Name)
					break INIT
				}
			}
			fmt.Fprintf(conn, "Group not found")
			continue
		}
		if input.Text() == "c" {
			fmt.Fprintf(conn, "Enter Group Name:")
			input.Scan()
			groupName := input.Text()
			g := NewGroup(groupName)
			_, err := DB.Exec("INSERT INTO groups (groupname) VALUES ($1)", groupName)
			if err != nil {
				fmt.Fprintf(conn, "Unable to create group\n")
				continue
			}
			go g.groupBroadcaster()
			g.Entering <- ch
			g.Messages <- user.username + " has arrived "
			MatchedGroup = g
			GROUPS[&g] = true
			fmt.Println(user.username, " has created the group ", g.Name)
			break
		}
	}
	go ClientWriter(conn, ch)
	ClientListener(conn, user, MatchedGroup, ch)
}

//ClientListener ...
func ClientListener(conn net.Conn, user User, g Group, ch chan string) {
	input := bufio.NewScanner(conn)
	for input.Scan() {
		text := input.Text()
		t := time.Now().Unix()
		msg := user.username + ": " + text
		_, err := DB.Exec(`INSERT INTO messages (sender,msg,time,groupname)
		VALUES ($1,$2,$3,$4)`, user.username, msg, t, g.Name)
		if err != nil {
			fmt.Fprintf(conn, "Unable to send message\n")
			continue
		}
		g.Messages <- msg
	}
	fmt.Println(user.username, " has left the group", g.Name)
	g.Leaving <- ch
	g.Messages <- user.username + " has left"
}

//ClientWriter ...
func ClientWriter(conn net.Conn, ch <-chan string) {
	for msg := range ch {
		fmt.Fprintln(conn, msg)
	}
}
