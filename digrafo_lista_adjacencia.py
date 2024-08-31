from collections import OrderedDict, deque
import heapq
class Digrafo_lista_adjacencia:

    def __init__(self,vertices):
        self.vertices=vertices
        self.num_vertices=self.n()
        self.lista_adj= list(vertices.values()) #lista de adjacencias
    def n(self):
        return len(self.vertices)
    def m(self):
        arestas = sum(len(lista_adj_vert)for lista_adj_vert in self.lista_adj)
        #A qtd de arestas será o somatório do tamanho das listas de adjacência de cada vértice na lista de adjacencias do grafo
        return arestas
    def viz(self, vertice):#retona os vizinhos
        vizinhos=[]
        for tupla in self.vertices[vertice]:#itera sobre as tuplas (vizinho,peso-da-aresta) na lista de adjacencia
            if len(tupla) == 2:#se a tupla tem tamanho 2, adiciona o vertice vizinho, sem o peso
                vizinhos.append(tupla[0])
        return vizinhos
        
    def d(self,vertice):#retorna o grau total do vértice do digrafo
       return(self.dEntrada(vertice)+self.dSaida(vertice))
       
    def dSaida(self,vertice):
        d=0
        d+=len(self.viz(vertice))#adiciona os Graus de saida
        return d
    def dEntrada(self, vertice):
        d=0
        for u in self.vertices:#adicona os graus de entrada
            for vizinho in self.viz(u):
                if vizinho==vertice:
                    d+=1
        return d
    def w(self, vertice_origem, vertice_final):
        for vizinho, peso in vertices[vertice_origem]:#Percorre a lista de adjacencias do vertice de origem
            if vertice_final==vizinho:#se um dos vizinhos do vertice vizinho for o destino da aresta, retorne o peso dessa aresta, pois ela existe
                return peso
        return None

    def mind(self):#calcula o grau total minimo
        grauMinimo = float('inf')# Atribui ao grau minimo o valor infinito por enquanto
        for vertice in self.vertices:
            if self.d(vertice)<grauMinimo:
                grauMinimo=self.d(vertice)
        return grauMinimo
    def maxd(self):#calcula o grau total maximo
        grauMaximo =0
        for vertice in self.vertices:
            if self.d(vertice)>grauMaximo:
                grauMaximo=self.d(vertice)
        return grauMaximo
        
    def bfs(self, vertice_origem):
        propVertices={}#dicionário que irá armazenar propriedades dos vértices relevantes para a BFS usando a mesma chave que vertices{}
        arvoreBuscaD=OrderedDict()#dicionário ordenado representando as distancias da arvore de busca
        arvoreBuscaPi=OrderedDict()
        #a escolha por dicionarios ordenados foi para permitir que a manipulação do conteudo, como transformar em lista, não altere a ordem
        for vertice in self.vertices:
            if vertice == vertice_origem:#para v pertecente a V - {s} (origem)
                continue
            if vertice not in propVertices:
                propVertices[vertice]={ 'cor': 'branco', 'd': float('inf'),'pi': None }#inicializa
        propVertices[vertice_origem]={'cor':'cinza', 'd':0, 'pi':None}#inicializa origem
        q=deque([vertice_origem])#fila

        while q:#usa o retorno booleano implicito para saber se a lista esvaziou
            u = q.popleft()#remove o elemento mais a esquerda da fila e atribui a U. A escolha por "deque" se deve ao fato do metodo deque.popleft() ter complexidade O(1) equanto usar list.pop(0) tem complexidade O(n) 
            for vizinho, peso in self.vertices[u]:
                if propVertices[vizinho]['cor']=='branco':
                    propVertices[vizinho]['cor']='cinza'
                    propVertices[vizinho]['d']= propVertices[u]['d'] +1
                    propVertices[vizinho]['pi']=u
                    q.append(vizinho)
            propVertices[u]['cor']='preto'
            arvoreBuscaD[u]=propVertices[u]['d']#Após o vertice ficar preto, adicionamos na lista de distancias
            arvoreBuscaPi[u]=propVertices[u]['pi']#Após o vertice ficar preto adicionamos na lista de vertices predecessores.
        return ((arvoreBuscaD, arvoreBuscaPi))

    def dfs(self):
        propVertices={}#dicionario 
        arvoreBuscaPi=OrderedDict()
        arvoreBuscaInicio=OrderedDict()
        arvoreBuscaFim=OrderedDict()
        tempo=0
        def busca_dfs(vertice):
            nonlocal tempo #estava tentando criar uma variavel local tempo sempre que eu rodava,
            tempo+=1#incrementa o tempo para os passos seguintes
            propV= propVertices[vertice]#propriedades do vertice atual
            propV['ini']=tempo#tempo de inicio do vertice atual, já incrementado
            propV['cor']='cinza'#recebe a cor cinza
            arvoreBuscaPi[vertice]=propV['pi']#adicona o pai do vertice atual na lista resultante de pais
            arvoreBuscaInicio[vertice]=propV['ini']#adiciona o tempo de inicio na lista resultante de tempos iniciais
            
            for vizinho, peso in self.vertices[vertice]:#para cada vizinho e peso no dicionario de vertices
                propU = propVertices[vizinho]#propriedades do vertice vizinho armazenados em variavel, repetidas buscas complexas
                if propU['cor']=='branco':
                    propU['pi']=vertice
                    busca_dfs(vizinho)
            tempo+=1#incrementa o tempo para finalizar o vertice
            propV['fim']=tempo#após as recurssões sucessivas para os vizinhos, todos os vizinhos já foram visitados, o vertice ja pode ser finalizado e ficar preto
            propV['cor']='preto'
            arvoreBuscaFim[vertice]=propV['fim']#adicona na lista de tempos finais

        for vertice in self.vertices:
            if vertice not in propVertices:
                propVertices[vertice]={ 'cor': 'branco','pi': None, 'ini':0, 'fim':0 }#inicializa as propriedades de cada vertice na est. de dados propVertices
        for vertice in self.vertices:
            if propVertices[vertice]['cor']=='branco':
                busca_dfs(vertice)
        return ((arvoreBuscaPi, arvoreBuscaInicio, arvoreBuscaFim))

    def relaxa(self,propVertices, vertice1, vertice2, peso):#Função de relaxamento auxiliar de bellman_ford e djisktra
        u = propVertices[vertice1]#2 variaveis criadas para reduzir a complexidade
        v = propVertices[vertice2]#
        w_u_v =peso# w(u,v) peso entre u e v
        if v['d'] >u['d'] + w_u_v:#se a distância a v for maior que a distância de u mais o peso da aresta entre eles
            v['d'] = u['d'] +w_u_v
            v['pi']= vertice1
    
    def inicializa_cam_min(self, origem):#Algoritimo que inicializa caminho minimo implementado para ser usado em dijkstra e bellman_ford
        propVertices={}#dicionario com as propriedades dos vertices necessárias para algoritimos de caminhos minimos, 'pi' e 'd'
        for vertice in self.vertices:
            if vertice not in propVertices:
                propVertices[vertice]={'d': float('inf'),'pi': None }
        propVertices[origem]['d']=0
        return propVertices
        
    def extrair_arestas(self):#A princípio, eu não iria colocar uma função como essa, mas tornou-se necessária para reduzir a complexidade do algoritimo de bellman_ford de O(VxVxE) para O(VxE)
        arestas=[]#arestas será uma lista de tuplas com 3 elementos: vertice origem, seu vizinho e o peso
        for vertice in self.vertices:#complexidade extrair_arestas O(VxE)
            for vizinho, peso in self.vertices[vertice]:
                arestas.append((vertice, vizinho, peso))
        return arestas

    def bellman_ford(self, origem):
        propVertices=self.inicializa_cam_min(origem)
        for i in range(self.num_vertices-1):#Complexidade O(VxE)
            for vertice, vizinho, peso in self.extrair_arestas():#pra cada aresta aplique a função de relaxamento
                self.relaxa(propVertices, vertice, vizinho, peso)

        
        for vertice, v, peso in self.extrair_arestas():
            if propVertices[v]['d']>propVertices[vertice]['d']+peso:
                return False,"Ciclo negativo detectado"

        return True, propVertices
    

    def dijkstra(self, origem):#Tive que usar heapq para pegar um heap pronto
        propVertices=self.inicializa_cam_min(origem)#inicializa
        q=[(0, origem)]#instancia a lista que será usada como heap queue
        while q:#utiliza o retorno implicito que dará falso qunado q for vazia
            d_u, u=heapq.heappop(q)#extrai a distancia minima, por padrão o heapq utiliza um heap de minimo, colocando o menor valor no topo
            if d_u>propVertices[u]['d']:
                continue
            for vizinho,peso in self.vertices[u]:
                propV= propVertices[vizinho]  #evitar buscas repetitivas
                d_antiga= propV['d']  #Guarda a distância antiga
                self.relaxa(propVertices, u, vizinho, peso)
                if propV['d']<d_antiga:#adiciona o vizinho na fila de prioridade se a distância foi atualizada
                    heapq.heappush(q,(propV['d'], vizinho))
        return propVertices
   
#casos de teste
# vertices={
#     '1':[('2', 2)],
#     '2':[('3', 99), ('5', 4)],
#     '3':[],
#     '4':[('1', 7), ('6', 17)],
#     '5':[('6', 1)],
#     '6':[('3', 90)]
# }
# digrafo = Digrafo_lista_adjacencia(vertices)

# print(f"quantidade de vértices:{digrafo.n()}")
# print(f"Quantidade de arestas:{digrafo.m()}")
# print(f"Vizinhos do vértice 2 : {digrafo.viz('2')}")
# print(f"grau do vértice 1: {digrafo.d('1')}")
# print(f"peso da aresta 2-3:{digrafo.w('2','3')}")
# print(f"grau total minimo do dirgafo:{digrafo.mind()}")
# print(f"grau total maximo do grafo:{digrafo.maxd()}")
# print(f"Lista de distancias da busca BFS{digrafo.bfs('4')[0]}, Lista de predecessores BFS:{digrafo.bfs('4')[1]}")
# print(f'Lista de predecessores DFS:{digrafo.dfs()[0]}, Lista de tempos iniciais DFS:{digrafo.dfs()[1]}, Lista de tempos finais:{digrafo.dfs()[2]}')
# print(f'Bellman-ford começando pelo vertice 4:{digrafo.bellman_ford('4')}')
# print(f'Dijkstra começando pelo vertice 4 :{digrafo.dijkstra('4')}')