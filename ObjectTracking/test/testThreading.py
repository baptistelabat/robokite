import threading

def affiche(nb, nom = ''):
    for i in range(nb): print nom, i

a = threading.Thread(None, affiche, None, (200,), {'nom':'thread a'})
b = threading.Thread(None, affiche, None, (200,), {'nom':'thread b'})
a.start()
b.start()
