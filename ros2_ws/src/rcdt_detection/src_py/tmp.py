brick_height = 0.07
brick_offset = 0.08

base_layer_size = 3
layer = 0
row_idx = 0

# triangle_max = (base_layer_size * (base_layer_size + 1)) / 2

while True:
    if layer >= base_layer_size:
        print("stopping")
        break

    x = 0.12 + ((row_idx) * brick_offset) + (layer * (brick_offset / 2))
    y = 0.5
    z = 0.00 + (layer * brick_height)

    print(row_idx, ":", x, y, z)
    row_idx += 1
    if row_idx >= base_layer_size - layer:
        row_idx = 0
        layer += 1
