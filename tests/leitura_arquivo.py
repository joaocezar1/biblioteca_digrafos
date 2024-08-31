from collections import OrderedDict
arquivo = open('USA-road-d.NY.gr/USA-road-d.NY.gr', 'r')
vertices=OrderedDict()
i=0
for linha in arquivo:
    if i<7:
        i+=1
        continue
    chars=linha.split()
    #Caso o vértice ainda não esteja na lista, cria-se uma lista de 
    #adjacencias para ele
    if chars[1] not in vertices:
        vertices[chars[1]] = []
    if chars[2] not in vertices:
        vertices[chars[2]] = []
    # Adiciona o vizinho a lista de adjacencia do vertice com o seu peso em uma tupla (vizinho,peso)
    vertices[chars[1]].append((chars[2], chars[3])) #como é dígrafo, apenas essa adjacencia é valida
    #vertices[chars[2]].append(chars[1])



arquivo.close()