package main

import (
	"flag"
)

func main() {
	var fileName = flag.String("file", "", "")
	var outputFileName = flag.String("output", "", "")

	flag.Parse()

	err := assemble(*fileName, *outputFileName)
	if err != nil {
		println(err.Error())
	}
}
