import requests

url = 'http://localhost:9090/select'
myobj = {"query": "PREFIX props: <https://w3id.org/props#> SELECT * WHERE {?inst props:Category 'Columns' .}"}

x = requests.post(url, json = myobj)

print(x.text)