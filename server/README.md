# Server

This is the server side of the project. The goal of the server is to provide querying of the data from the graph database as well as updating the database. The server is written in relativly young language named Golang (Go). One could ask why Go and the answer is the entusiasm for learning new things as well as a very high demand for Go in the industry lately. It is one of the fastest growing language with very good documentation! It provides a really good platform for conncurent programming and finally it is low-level language. The main thing that engineers managed to do with Go is fast compiling (this is one of the benefits over C++). The resources for learning Go are documentation and tour of Go on their website (https://go.dev/tour/welcome/1).

P.S. the best thing about Go is that it is open source!

## What is server?

Server is a computer. It is very important part of every web application, because it has a communication with a database, it executes code that is high demanding... Server communicate with their clients using APIs. For example when a user wants to connect to their account on Facebook, in the moment when user clicks "Log in" button the API is sent to the one of the Facebook servers, where the server checks if the username and password are the same as in the database and sends the respond also as an API.

In our project it should be like: Robot sends API to get the needed data, server queries the database and sends the response back to the robot. The database should be updated similarly.

APIs are sent using the HTTP protocol, which uses the IP addresses of the server and robot. The IP address is unique for every machine in the world.

## How to use the server?

You should open this folder in a new workspace and run the following command:

```bash 
$ go run .
```
This command should be running and waiting for http requests. If you take a look at the code the function ListenAndServe waits for the request and responds in the proper way. How it will respond it depends on the handlers.

As the server is a computer, one wants to run several processes. Maybe the creator wants to use these processes and make them to communicate. Exactly this happens in this case, because we want that our server to communicate with the database which is on the port 7200 when you run it. Behind the scenes when we run the database, actually the database server is run. So every time when robot sends API to our server, it is forwarded to the database server. The APIs for graphDB can be found in their documentation https://graphdb.ontotext.com/documentation/10.0/pdf/GraphDB.pdf#24e. We can not just make a client on the robot side that will send queries to the graphDB server, because this is not secure and robot does not have enough power to process the data fastly. Therefore we need out own server that will communicate with both robot and database.

## Quering database

The GET request (select queries) can be done using the following command:
```bash
curl -X GET -H "Content-Type: application/json" -d '{"query": "<select_query>"}' http://localhost:9090/
```

To make quering easier we send json with a key "query" and a following query. From the code you can send GET request and provide the json.