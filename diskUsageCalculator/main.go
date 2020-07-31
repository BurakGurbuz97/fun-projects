package main

import (
	"flag"
	"fmt"
	"io/ioutil"
	"os"
	"path/filepath"
	"sync"
	"time"
)

var direct = make(chan directInfo, 50)

func walkDir(dir string, n *sync.WaitGroup, fileSizes chan<- int64, direct chan<- directInfo) {
	defer n.Done()
	//Recursive walkdir
	var currentDirSize int64
	for _, entry := range dirents(dir) {
		if entry.IsDir() {
			//inner directory
			subdir := filepath.Join(dir, entry.Name())
			n.Add(1)
			go walkDir(subdir, n, fileSizes, direct)
		} else {
			sz := entry.Size()
			currentDirSize += sz
			fileSizes <- sz
		}
	}
	if *directories {
		sz := currentDirSize
		direct <- directInfo{dir, sz}
	}
}

var sema = make(chan struct{}, 20)

func dirents(dir string) []os.FileInfo {
	sema <- struct{}{}
	defer func() { <-sema }()
	entries, err := ioutil.ReadDir(dir)
	if err != nil {
		fmt.Fprintf(os.Stdin, "du1: %v\n", err)
	}
	return entries
}

func printDiskUsage(nfiles, nbytes int64) {
	fmt.Fprintf(os.Stdout, "\n%d files %.1f gb\n", nfiles, float64(nbytes)/1e9)
}

var verbose = flag.Bool("v", false, "show verbose progress messages")
var directories = flag.Bool("d", false, "list by directories")

type directInfo struct {
	name string
	size int64
}

func main() {
	//parse commandline arguments
	flag.Parse()
	roots := flag.Args()
	if len(roots) == 0 {
		roots = []string{"."}
	}

	//Print verbose messagess if -v
	var tick <-chan time.Time
	if *verbose {
		tick = time.Tick(500 * time.Millisecond)
	} else if *directories {
	} else {
		// Animation routine
		go func() {
			for {
				for _, c := range `-\|/` {
					fmt.Printf("\r calculating %c", c)
					time.Sleep(100 * time.Millisecond)
				}
			}
		}()
	}

	fileSizes := make(chan int64)
	var n sync.WaitGroup
	for _, root := range roots {
		n.Add(1)
		go walkDir(root, &n, fileSizes, direct)
	}
	go func() {
		n.Wait()
		close(fileSizes)
		close(direct)
	}()

	// Print the results
	var nfiles, nbytes int64
loop:
	for {
		select {
		case size, ok := <-fileSizes:
			if !ok {
				break loop
			}
			nfiles++
			nbytes += size
		case info, ok := <-direct:
			if !ok {
				break loop
			}
			fmt.Printf("%s  %.1f mb\n", info.name, float64(info.size)/1024)
		case <-tick:
			printDiskUsage(nfiles, nbytes)
		}

	}
	printDiskUsage(nfiles, nbytes)
}
