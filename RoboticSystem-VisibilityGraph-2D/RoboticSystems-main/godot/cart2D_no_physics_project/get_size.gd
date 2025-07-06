extends MeshInstance3D


func _ready():
	var mesh_instance = $MeshInstance3D  # Assicurati che il percorso del nodo sia corretto
	if mesh_instance and mesh_instance.mesh:
		var aabb = mesh_instance.mesh.get_aabb()  # Ottieni l'AABB della mesh
		print("Dimensione AABB:", aabb.size)  # Stampa le dimensioni dell'AABB
	else:
		push_warning("Mesh o nodo non valido.")
