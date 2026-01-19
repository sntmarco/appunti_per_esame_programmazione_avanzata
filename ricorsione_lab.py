# =====================================================
# Schema base ricorsione
# =====================================================

def calcola_soluzione_ricorsione():
    self.best_solution = []
    self.best_value = -inf
    parziale = []
    ricorsione(parziale, valore_iniziale)
    return self.best_solution, self.best_value

def ricorsione(parziale, valore_corrente):

    # CASO BASE
    if soluzione_completa(parziale):
        if valore_corrente migliore di best_value:
            aggiorna best_solution e best_value
        return

    # PASSO RICORSIVO
    for scelta in scelte_possibili(parziale):
        if scelta_ammissibile(parziale, scelta):
            parziale.append(scelta)
            ricorsione(parziale, nuovo_valore)
            parziale.pop()



# =====================================================
# Ricorsione Jolly
# =====================================================

def calcola_soluzione_ricorsione():
    self.best_solution = []
    self.best_value = -1  # oppure 0 / -inf a seconda del problema

    parziale = []         # soluzione costruita finora
    valore = 0            # valore della soluzione parziale

    ricorsione(parziale, valore)

    return self.best_solution, self.best_value

def ricorsione(parziale, valore_corrente):

    # --- CASO BASE ---
    if soluzione_completa(parziale):
        if valore_corrente > self.best_value:
            self.best_value = valore_corrente
            self.best_solution = parziale[:]
        return
    
    # --- PASSO RICORSIVO ---
    for scelta in scelte_possibili(parziale):

        if scelta_ammissibile(parziale, scelta):

            parziale.append(scelta)
            nuovo_valore = valore_corrente + costo(scelta, parziale)

            ricorsione(parziale, nuovo_valore)

            parziale.pop()

def soluzione_completa(parziale):
    # es: len(parziale) == L
    # es: parziale[-1] == nodo_finale
    # es: nessun'altra scelta possibile (sempre vera)
    pass

def scelte_possibili(parziale):
    # es: nodi adiacenti
    # es: album della componente connessa
    # es: vicini con peso crescente
    pass

def scelta_ammissibile(parziale, scelta):
    # es: scelta non già in parziale
    # es: peso totale <= max
    # es: arco con peso crescente
    return True

def costo(scelta, parziale):
    # es: peso arco
    # es: durata album
    # es: distanza geografica
    return 0

#🟢 BikeStore
#soluzione_completa → len(parziale) == L and parziale[-1] == end
#scelte_possibili → successori del nodo corrente
#scelta_ammissibile → nodo non già visitato
#costo → peso dell’arco

#🟢 iTunes
#soluzione_completa → sempre vera (ogni set è valido)
#scelte_possibili → album della componente
#scelta_ammissibile → durata totale <= max
#costo → durata album

#🟢 UFO Sightings
#soluzione_completa → nessun vicino ammissibile
#scelte_possibili → archi uscenti
#scelta_ammissibile → peso arco crescente
#costo → distanza tra stati


#============================================================================================================================================================

# =====================================================
# SE_BikeStore (nodoInizio -> nodoFine, lunghezza(nodi) = L)
# =====================================================

#si implementi un algoritmo ricorsivo che identifichi un cammino ottimo tale per cui:

#Il cammino parta dal nodo identificato come “Prodotto Iniziale” e termini nel nodo identificato come “Prodotto Finale”;
#La lunghezza del cammino sia pari a L, valore numerico fornito dall’utente nel campo “Lunghezza Cammino”;
#Il cammino attraversi gli archi rispettando i versi;
#Un nodo non può essere attraversato più volte;
#La somma dei pesi degli archi deve essere massima. Identificato il percorso, si visualizzino nella GUI la sequenza di nodi e la somma dei pesi degli archi.

def get_best_path(self, lungh, start, end):
        self.best_path = []
        self.best_score = 0
        parziale = [start]
        self._ricorsione(parziale, lungh, start, end)
        return self.best_path, self.best_score

def _ricorsione(self, parziale, lungh, start, end):
        if len(parziale) == lungh:
            if  parziale[-1] == end and self._get_score(parziale) > self.best_score:
                self.best_score = self._get_score(parziale)
                self.best_path = copy.deepcopy(parziale)
            return

        for n in self.G.successors(parziale[-1]):
            if n not in parziale:
                parziale.append(n)
                self._ricorsione(parziale, lungh, start, end)
                parziale.pop()

def _get_score(self, parziale):
        score = 0
        for i in range(1, len(parziale)):
            score += self.G[parziale[i-1]][parziale[i]]["weight"]
        return score



# =====================================================
# SE_Itunes (Peso_nodi < PesoMax)
# =====================================================

#utilizzare un algoritmo ricorsivo per estrarre un set di album dal grafo che abbia le seguenti caratteristiche:

#Includa "a1";
#Includa solo album appartenenti alla stessa componente connessa di "a1";
#Includa il maggior numero possibile di album;
#Abbia una durata complessiva, definita come la somma della durata degli album in esso contenuti, non superiore "dTOT".

def get_component(self, album):
        """Restituisce la componente connessa di un album"""
        if album not in self.G:
            return []
        return list(nx.node_connected_component(self.G, album))

def compute_best_set(self, start_album, max_duration):
        """Ricerca ricorsiva del set massimo di album nella componente connessa"""
        component = self.get_component(start_album)
        self.soluzione_best = []
        self._ricorsione(component, [start_album], start_album.duration, max_duration)
        return self.soluzione_best

def _ricorsione(self, albums, current_set, current_duration, max_duration):
        if len(current_set) > len(self.soluzione_best):
            self.soluzione_best = current_set[:]

        for album in albums:
            if album in current_set:
                continue
            new_duration = current_duration + album.duration
            if new_duration <= max_duration:
                current_set.append(album)
                self._ricorsione(albums, current_set, new_duration, max_duration)
                current_set.pop()


# =====================================================
# SE_ufo_sightings (massima distanza fra nodi)
# =====================================================

#Dato il grafo costruito al punto precedente, si vuole identificare un percorso semplice che massimizza la distanza tra stati con archi di peso sempre crescente.

#Alla pressione del pulsante "Calcola Percorso", avviare una procedura ricorsiva per calcolare il percorso ottimo.
#Visualizzare nella GUI:
#il percorso trovato (elenco di stati);
#il peso di ciascun arco;

def compute_path(self):
        self.path = []
        self.path_edge = []
        self.sol_best = 0

        partial = []
        for n in self.get_nodes():
            partial.clear()
            partial.append(n)
            self._ricorsione(partial, [])

def _ricorsione(self, partial, partial_edge):
        n_last = partial[-1]

        neighbors = self.get_admissible_neighbs(n_last, partial_edge)

        if len(neighbors) == 0:
            weight_path = self.compute_weight_path(partial_edge)
            if weight_path > self.sol_best:
                self.sol_best = weight_path + 0.0
                self.path = partial[:]
                self.path_edge = partial_edge[:]
            return

        for n in neighbors:
            partial_edge.append((n_last, n, self.G.get_edge_data(n_last, n)['weight']))
            partial.append(n)

            self._ricorsione(partial, partial_edge)

            partial.pop()
            partial_edge.pop()

def get_admissible_neighbs(self, n_last, partial_edges):
        all_neigh = self.G.edges(n_last, data=True)
        result = []
        for e in all_neigh:
            if len(partial_edges) != 0:
                if e[2]["weight"] > partial_edges[-1][2]:
                    result.append(e[1])
            else:
                result.append(e[1])
        return result

def compute_weight_path(self, mylist):
        weight = 0
        for e in mylist:
            weight += distance.geodesic((e[0].lat, e[0].lng),
                                        (e[1].lat, e[1].lng)).km
        return weight

def get_distance_weight(self, e):
        return distance.geodesic((e[0].lat, e[0].lng),
                                 (e[1].lat, e[1].lng)).km



# =====================================================
# SE_Baseball (percorso con peso max a partire da un nodo)
# =====================================================

#Partendo dal grafo calcolato nel punto precedente, alla pressione del pulsante “Percorso” si implementi una procedura ricorsiva che calcoli un percorso di peso massimo avente le seguenti caratteristiche:

#Il punto di partenza è il vertice selezionato al punto 1.5;
#Ogni vertice può comparire una sola volta;
#Il peso degli archi nel percorso deve essere strettamente decrescente.
#Per limitare i tempi di esecuzione, la procedura ricorsiva deve considerare solo i primi K archi adiacenti ordinati per peso decrescente (ad esempio K=3), cioè ogni vertice esplora al massimo i K vicini più pesanti che rispettano il vincolo decrescente.
#Al termine della ricerca, si visualizzi, nella seconda area di testo (txt_risultato), l’elenco dei vertici ed i pesi degli archi incontrati, oltre al peso totale del percorso ottimo.

def get_neighbors(self, team):
        vicini = []
        for n in self.G.neighbors(team):
            w = self.G[team][n]["weight"]
            vicini.append((n, w))
        return sorted(vicini, key=lambda x: x[1], reverse=True)

def compute_best_path(self, start):
        """Calcola percorso di peso massimo con archi strettamente decrescenti"""
        self.best_path = []
        self.best_weight = 0
        self._ricorsione([start], 0, float("inf"))
        return self.best_path, self.best_weight

def _ricorsione(self, path, weight, last_edge_weight):
        last = path[-1]
        if weight > self.best_weight:
            self.best_weight = weight
            self.best_path = path.copy()

        vicini = self.get_neighbors(last)
        neighbors = []
        counter = 0
        for node, edge_w in vicini:
            if node in path:
                continue
            if edge_w <= last_edge_weight:
                neighbors.append((node, edge_w))
                counter += 1
                if counter == self.K:
                    break

        for node, edge_w in neighbors:
            path.append(node)
            self._ricorsione(path, weight + edge_w, edge_w)
            path.pop()



# =====================================================
# Lab13_SE (cammino massimo con vincolo)
# =====================================================

#A partire dal grafo calcolato al punto precedente, alla pressione del tasto “Ricerca Cammino”, 
#si avvii una procedura di ricerca ricorsiva per determinare il cammino di vertici (cromosomi) 
#del grafo che massimizza il costo complessivo, calcolato come somma dei costi 
#associati agli archi (o ai vertici) che sia composto esclusivamente da archi di peso >S. 
#Il costo del cammino sarà valutata dalla somma dei pesi degli archi incontrati.

def ricerca_cammino(self, t):
        self.soluzione_best.clear()

        for n in self.get_nodes():
            partial = []
            partial_edges = []

            partial.append(n)
            self.ricorsione(partial, partial_edges, t)

        print("final", len(self.soluzione_best), [i[2]["weight"] for i in self.soluzione_best])

def ricorsione(self, partial_nodes, partial_edges, t):
        n_last = partial_nodes[-1]
        neigh = self._get_admissible_neighbors(n_last, partial_edges, t)

        # stop
        if len(neigh) == 0:
            weight_path = self.compute_weight_path(partial_edges)
            weight_path_best = self.compute_weight_path(self.soluzione_best)
            if weight_path > weight_path_best:
                self.soluzione_best = partial_edges[:]
            return

        for n in neigh:
            print("...")
            partial_nodes.append(n)
            partial_edges.append((n_last, n, self.G.get_edge_data(n_last, n)))
            self.ricorsione(partial_nodes, partial_edges, t)
            partial_nodes.pop()
            partial_edges.pop()

def _get_admissible_neighbors(self, node, partial_edges, soglia):
        result = []
        for u, v, data in self.G.out_edges(node, data=True):
            if data["weight"] > soglia:
                # controllo SOLO l'arco diretto
                if (u, v) not in [(x[0], x[1]) for x in partial_edges]:
                    result.append(v)
        return result

def compute_weight_path(self, mylist):
        weight = 0
        for e in mylist:
            weight += e[2]['weight']
        return weight



# =====================================================
# Ricorsione universale per cammino massimo/minimo con vincoli
# =====================================================

def calcola_percorso_ottimo(self):
        self.best_solution = []
        self.best_value = 0 #float("inf") se percorso min - oppure float("-inf") se percorso max

        for nodo in self.G.nodes():
            self._ricorsione(
                parziale=[nodo],
                visitati={nodo},
                valore_corrente=0,
                ultimo_peso=-1
            )
        self.stampa_percorso()
        return self.best_solution, self.best_value

def _ricorsione(self, parziale, visitati, valore_corrente, ultimo_peso):

        # aggiorna sempre il best (> percorso max, < percorso min) oppure (len(parziale) == L)  oppure  (parziale[-1] == nodo_finale)
        if valore_corrente > self.best_value:
            self.best_value = valore_corrente
            self.best_solution = parziale[:]

        nodo_corrente = parziale[-1]

        for vicino in self.G.neighbors(nodo_corrente):

            if vicino in visitati:
                continue

            peso = self.G[nodo_corrente][vicino]["weight"]

            # vincolo: pesi crescenti
            if peso <= ultimo_peso:
                continue
            
            # vincolo: pesi maggiori soglia
            if peso < self.soglia:
                continue

            parziale.append(vicino)
            visitati.add(vicino)

            self._ricorsione(
                parziale,
                visitati,
                valore_corrente + peso,
                peso
            )

            visitati.remove(vicino)
            parziale.pop()

def stampa_percorso(self):
        for i in range(len(self.best_solution) - 1):
            u = self.best_solution[i]
            v = self.best_solution[i + 1]

            peso = self.G[u][v]["weight"]
            print(f"{u} -> {v} | peso={peso}")