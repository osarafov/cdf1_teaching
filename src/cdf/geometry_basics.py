import Rhino.Geometry as rg
import math
import random
from System.Drawing import Color


def create_cube_grid(cell_size, plane, cells_x, cells_y, cells_z):
    """Create points and lines describing an oriented 3D cube grid.

    The plane origin and axes define the grid's origin and local coordinate
    system. Returns unique grid vertices, cell-center points, and unique edge
    lines, in that order. Cell indices increase along the plane's local axes.
    """
    # ERROR: cell_size must be positive.
    if cell_size <= 0:
        raise ValueError("cell_size must be positive.")
    # ERROR: plane must be a valid Rhino.Geometry.Plane.
    if plane is None or not plane.IsValid:
        raise ValueError("plane must be a valid Rhino.Geometry.Plane.")

    counts = (cells_x, cells_y, cells_z)
    # ERROR: Cell counts must be positive integers.
    if any(
        isinstance(count, bool) or int(count) != count or count < 1
        for count in counts
    ):
        raise ValueError("Cell counts must be positive integers.")

    # Convert validated counts to integers for the range loops below.
    cells_x, cells_y, cells_z = (int(count) for count in counts)

    # Convert grid-local coordinates into Rhino world-space points.
    def point_at(x, y, z):
        return plane.PointAt(x * cell_size, y * cell_size, z * cell_size)

    # Include the outer boundary, so each axis has one more vertex than cells.
    vertices = [
        point_at(x, y, z)
        for x in range(cells_x + 1)
        for y in range(cells_y + 1)
        for z in range(cells_z + 1)
    ]
    # Place one point halfway along each cell in all three directions.
    centers = [
        point_at(x + 0.5, y + 0.5, z + 0.5)
        for x in range(cells_x)
        for y in range(cells_y)
        for z in range(cells_z)
    ]

    # Build each shared grid edge once, grouped by its direction.
    edges = []
    # These lines run parallel to the local X axis.
    for x in range(cells_x):
        for y in range(cells_y + 1):
            for z in range(cells_z + 1):
                edges.append(rg.Line(point_at(x, y, z), point_at(x + 1, y, z)))
    # These lines run parallel to the local Y axis.
    for x in range(cells_x + 1):
        for y in range(cells_y):
            for z in range(cells_z + 1):
                edges.append(rg.Line(point_at(x, y, z), point_at(x, y + 1, z)))
    # These lines run parallel to the local Z axis.
    for x in range(cells_x + 1):
        for y in range(cells_y + 1):
            for z in range(cells_z):
                edges.append(rg.Line(point_at(x, y, z), point_at(x, y, z + 1)))

    # Return the geometry lists and cell size for separate Grasshopper outputs.
    return vertices, centers, edges, cell_size


def _scaled_dimension(cell_size, units, dimension_name):
    """Convert a positive whole number of grid cells into a model length."""
    # ERROR: cell_size must be positive.
    if cell_size <= 0:
        raise ValueError("cell_size must be positive.")
    # ERROR: The requested dimension must be a positive integer number of cells.
    if isinstance(units, bool) or int(units) != units or units < 1:
        raise ValueError("{} must be a positive integer.".format(dimension_name))

    # Scale the cell count into the document's model units.
    return int(units) * cell_size


def _extrusion_frame(location, direction, height):
    """Return a base plane and extrusion vector centered on location."""
    # ERROR: direction must be a non-zero Rhino vector.
    if direction is None or direction.IsZero:
        raise ValueError("direction must be a non-zero Rhino.Geometry.Vector3d.")

    # Normalize direction so the requested height remains the extrusion length.
    unit_direction = rg.Vector3d(direction)
    unit_direction.Unitize()
    extrusion_vector = unit_direction * height

    # Locate the base half a height behind the center along the extrusion axis.
    base_center = location - unit_direction * (height / 2.0)
    base_plane = rg.Plane(base_center, unit_direction)
    return base_plane, extrusion_vector


def place_sphere(location, cell_size, base_diameter):
    """Create a sphere centered at location and return its Brep."""
    # ERROR: location must be a valid Rhino point.
    if location is None or not location.IsValid:
        raise ValueError("location must be a valid Rhino.Geometry.Point3d.")

    # Scale the integer diameter from grid cells into model units.
    diameter = _scaled_dimension(cell_size, base_diameter, "base_diameter")
    sphere = rg.Sphere(location, diameter / 2.0)
    brep = sphere.ToBrep()

    return brep


def place_cylinder(location, cell_size, base_diameter, height, direction):
    """Create a centered cylinder oriented along direction."""
    # ERROR: location must be a valid Rhino point.
    if location is None or not location.IsValid:
        raise ValueError("location must be a valid Rhino.Geometry.Point3d.")

    # Convert the grid-unit diameter and height to model-space lengths.
    diameter = _scaled_dimension(cell_size, base_diameter, "base_diameter")
    extrusion_height = _scaled_dimension(cell_size, height, "height")
    base_plane, _ = _extrusion_frame(location, direction, extrusion_height)

    # Build a capped circular extrusion with the requested base diameter.
    circle = rg.Circle(base_plane, diameter / 2.0)
    brep = rg.Cylinder(circle, extrusion_height).ToBrep(True, True)

    return brep


def place_prism(location, cell_size, base_diameter, vertices, height,
                direction, grid_plane):
    """Create a centered regular prism with the requested inscribed diameter."""
    # ERROR: location must be a valid Rhino point.
    if location is None or not location.IsValid:
        raise ValueError("location must be a valid Rhino.Geometry.Point3d.")
    # ERROR: A regular prism base needs at least three vertices.
    if isinstance(vertices, bool) or int(vertices) != vertices or vertices < 3:
        raise ValueError("vertices must be an integer of at least 3.")
    # ERROR: grid_plane must be a valid Rhino.Geometry.Plane.
    if grid_plane is None or not grid_plane.IsValid:
        raise ValueError("grid_plane must be a valid Rhino.Geometry.Plane.")

    # Convert the grid-unit diameter and height to model-space lengths.
    diameter = _scaled_dimension(cell_size, base_diameter, "base_diameter")
    extrusion_height = _scaled_dimension(cell_size, height, "height")
    base_plane, _ = _extrusion_frame(location, direction, extrusion_height)

    # Project the grid's local X axis onto the prism base plane.
    unit_direction = rg.Vector3d(direction)
    unit_direction.Unitize()
    base_x = rg.Vector3d(grid_plane.XAxis)
    dot_product = (
        unit_direction.X * base_x.X
        + unit_direction.Y * base_x.Y
        + unit_direction.Z * base_x.Z
    )
    base_x -= unit_direction * dot_product
    # ERROR: The grid X axis must not be parallel to the extrusion direction.
    if base_x.IsZero:
        base_x = rg.Vector3d(grid_plane.YAxis)
        dot_product = (
            unit_direction.X * base_x.X
            + unit_direction.Y * base_x.Y
            + unit_direction.Z * base_x.Z
        )
        base_x -= unit_direction * dot_product
    # ERROR: Neither grid-plane axis can orient the prism base.
    if base_x.IsZero:
        raise ValueError("grid_plane cannot define an orientation perpendicular to direction.")
    base_x.Unitize()
    base_y = rg.Vector3d.CrossProduct(unit_direction, base_x)
    oriented_plane = rg.Plane(base_plane.Origin, base_x, base_y)

    # Expand the circumradius so the requested circle fits inside the polygon.
    circumradius = diameter / (2.0 * math.cos(math.pi / int(vertices)))
    side_phase = -math.pi / 2.0 - math.pi / int(vertices)
    polygon_points = [
        oriented_plane.PointAt(
            circumradius * math.cos(
                side_phase + 2.0 * math.pi * index / int(vertices)
            ),
            circumradius * math.sin(
                side_phase + 2.0 * math.pi * index / int(vertices)
            ),
        )
        for index in range(int(vertices))
    ]
    polygon_points.append(polygon_points[0])

    # Extrude the closed polygon along the base plane normal and cap both ends.
    profile = rg.Polyline(polygon_points).ToNurbsCurve()
    extrusion = rg.Extrusion.Create(profile, extrusion_height, True)
    # ERROR: Rhino must successfully create the capped prism Brep.
    if extrusion is None:
        raise RuntimeError("Rhino could not create the prism Brep.")
    brep = extrusion.ToBrep()

    return brep


def create_sphere_shells(
    sphere_brep,
    cell_size,
    sphere_count,
    offset_cells,
    prism_brep=None,
    vertex_index=0,
    point_index=0,
):
    """Create concentric spherical shells from an input sphere Brep.

    The input sphere is the smallest sphere. Each next sphere increases its
    radius by offset_cells * cell_size. The function subtracts each smaller
    sphere from the next larger sphere and returns the shell Breps. If a prism
    Brep is provided, a duplicate is moved to each generated point and cut
    from every retained shell. Each shell also receives a random Bauhaus-style
    primary color and material properties.
    """
    # ERROR: sphere_brep must be a valid Rhino Brep.
    if sphere_brep is None or not sphere_brep.IsValid:
        raise ValueError("sphere_brep must be a valid Rhino.Geometry.Brep.")
    # ERROR: cell_size must be positive.
    if cell_size <= 0:
        raise ValueError("cell_size must be positive.")
    # ERROR: sphere_count must be an integer of at least 2.
    if (
        isinstance(sphere_count, bool)
        or int(sphere_count) != sphere_count
        or sphere_count < 2
    ):
        raise ValueError("sphere_count must be an integer of at least 2.")
    # ERROR: offset_cells must be a positive integer.
    if (
        isinstance(offset_cells, bool)
        or int(offset_cells) != offset_cells
        or offset_cells < 1
    ):
        raise ValueError("offset_cells must be a positive integer.")
    # Use point 0 when the optional Grasshopper input is left unwired.
    if point_index is None:
        point_index = 0
    # ERROR: point_index must select one of the 100 generated points.
    if (
        isinstance(point_index, bool)
        or int(point_index) != point_index
        or point_index < 0
        or point_index > 99
    ):
        raise ValueError("point_index must be an integer from 0 to 99.")

    sphere_count = int(sphere_count)
    point_index = int(point_index)
    offset_distance = int(offset_cells) * cell_size

    # Use the input sphere's bounding box to recover its center and radius.
    bounding_box = sphere_brep.GetBoundingBox(True)
    center = bounding_box.Center
    radius = bounding_box.Max.X - center.X
    # ERROR: sphere_brep must have a positive spherical radius.
    if radius <= 0:
        raise ValueError("sphere_brep must have a positive spherical radius.")

    # Validate the optional prism only when a cutout has been requested.
    if prism_brep is not None:
        # Use the first prism vertex when the optional input is left unwired.
        if vertex_index is None:
            vertex_index = 0
        # ERROR: prism_brep must be a valid Rhino Brep.
        if not prism_brep.IsValid:
            raise ValueError("prism_brep must be a valid Rhino.Geometry.Brep.")
        # ERROR: vertex_index must identify a prism Brep vertex.
        if (
            isinstance(vertex_index, bool)
            or int(vertex_index) != vertex_index
            or vertex_index < 0
            or vertex_index >= prism_brep.Vertices.Count
        ):
            raise ValueError("vertex_index is outside the prism's vertex list.")
        vertex_index = int(vertex_index)

    # Create concentric sphere Breps, starting with the input sphere's radius.
    spheres = [
        rg.Sphere(center, radius + index * offset_distance).ToBrep()
        for index in range(sphere_count)
    ]
    # Generate points inside the complete geometry, including the outermost sphere.
    cutout_point = _generate_shell_points(
        center, radius + (sphere_count - 1) * offset_distance, point_index
    )

    shells = []
    # Keep every other radial band: shell, void, shell, void, and so on.
    for index in range(0, sphere_count - 1, 2):
        # Build a hollow Brep directly because nested spheres do not intersect.
        outer = spheres[index + 1]
        inner = spheres[index]
        inner.Flip()
        outer.Append(inner)
        # ERROR: Rhino must successfully create each spherical shell.
        if not outer.IsValid:
            raise RuntimeError(
                "Rhino could not create spherical shell {}.".format(index)
            )
        shells.append(outer)

    # Assign randomized color/material combinations without adjacent repeats.
    shells = _cut_shells_with_prism(
        shells, prism_brep, cutout_point, vertex_index, cell_size
    )
    return _add_shell_materials(shells)


def _add_shell_materials(shells):
    """Assign parallel diffuse, specular, emission, transparency, and shine lists."""
    color_options = {
        "red": Color.FromArgb(215, 38, 61),
        "blue": Color.FromArgb(29, 78, 216),
        "yellow": Color.FromArgb(242, 201, 76),
    }
    material_options = ("regular plaster", "tinted glass", "colored metallic")
    combinations = [
        (color_name, color, material_name)
        for color_name, color in color_options.items()
        for material_name in material_options
    ]
    diffuse = []
    specular = []
    emission = []
    transparency = []
    shine = []
    previous_color = None
    previous_material = None
    for _ in shells:
        # Choose only combinations that differ from the previous shell in both dimensions.
        candidates = [
            combination
            for combination in combinations
            if combination[0] != previous_color
            and combination[2] != previous_material
        ]
        random.shuffle(candidates)
        color_name, color, material_name = candidates[0]
        # Return one material-property value per shell for a material component.
        diffuse.append(color)
        if material_name == "tinted glass":
            specular.append(Color.FromArgb(230, 245, 255))
            emission.append(Color.FromArgb(20, 20, 20))
            transparency.append(0.92)
            shine.append(0.98)
        elif material_name == "colored metallic":
            specular.append(Color.FromArgb(255, 255, 255))
            emission.append(Color.Black)
            transparency.append(0.0)
            shine.append(1.0)
        else:
            specular.append(Color.FromArgb(35, 35, 35))
            emission.append(Color.Black)
            transparency.append(0.0)
            shine.append(0.03)
        previous_color = color_name
        previous_material = material_name

    return shells, diffuse, specular, emission, transparency, shine


def _cut_shells_with_prism(shells, prism_brep, point, vertex_index, cell_size):
    """Move a prism vertex to point and subtract the prism from each shell."""
    if prism_brep is None:
        return shells

    # ERROR: prism_brep must be a valid Rhino Brep.
    if not prism_brep.IsValid:
        raise ValueError("prism_brep must be a valid Rhino.Geometry.Brep.")
    # ERROR: vertex_index must identify a prism Brep vertex.
    if (
        isinstance(vertex_index, bool)
        or int(vertex_index) != vertex_index
        or vertex_index < 0
        or vertex_index >= prism_brep.Vertices.Count
    ):
        raise ValueError("vertex_index is outside the prism's vertex list.")

    vertex_index = int(vertex_index)
    cut_shells = []
    tolerance = max(cell_size * 1e-6, 1e-6)
    for shell in shells:
        cutter = prism_brep.DuplicateBrep()
        source_point = cutter.Vertices[vertex_index].Location
        cutter.Transform(rg.Transform.Translation(point - source_point))
        shell_box = shell.GetBoundingBox(True)
        cutter_box = cutter.GetBoundingBox(True)
        if (
            shell_box.Max.X < cutter_box.Min.X
            or cutter_box.Max.X < shell_box.Min.X
            or shell_box.Max.Y < cutter_box.Min.Y
            or cutter_box.Max.Y < shell_box.Min.Y
            or shell_box.Max.Z < cutter_box.Min.Z
            or cutter_box.Max.Z < shell_box.Min.Z
        ):
            # The prism is outside this shell, so preserve the shell unchanged.
            cut_shells.append(shell)
            continue
        cut_shell = rg.Brep.CreateBooleanDifference(
            [shell], [cutter], tolerance
        )
        # A null result means the prism does not cut this shell.
        if cut_shell is None or len(cut_shell) == 0:
            cut_shells.append(shell)
        else:
            cut_shells.extend(cut_shell)
    return cut_shells


def _generate_shell_points(center, radius, point_index):
    """Generate 100 radially increasing points with 10 percent outer clearance."""
    if point_index is None:
        point_index = 0
    # ERROR: point_index must select one of the 100 generated points.
    if (
        isinstance(point_index, bool)
        or int(point_index) != point_index
        or point_index < 0
        or point_index > 99
    ):
        raise ValueError("point_index must be an integer from 0 to 99.")

    generated_points = [center]
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))
    for index in range(1, 100):
        radius_ratio = 0.9 * (index / 99.0) ** (1.0 / 3.0)
        # Avoid placing the final point at a pole, where it can look central
        # in a viewport even though its radial distance is correct.
        height_ratio = 1.0 - 2.0 * index / 100.0
        radial_distance = radius * radius_ratio
        radial_radius = math.sqrt(max(0.0, 1.0 - height_ratio ** 2))
        angle = golden_angle * index
        generated_points.append(
            rg.Point3d(
                center.X + radial_distance * radial_radius * math.cos(angle),
                center.Y + radial_distance * radial_radius * math.sin(angle),
                center.Z + radial_distance * height_ratio,
            )
        )
    return generated_points[int(point_index)]


def _generate_cylinder_point(center, radius, half_height, point_index):
    """Generate one of 100 points inside a cylindrical volume."""
    if point_index is None:
        point_index = 0
    # ERROR: point_index must select one of the 100 generated points.
    if (
        isinstance(point_index, bool)
        or int(point_index) != point_index
        or point_index < 0
        or point_index > 99
    ):
        raise ValueError("point_index must be an integer from 0 to 99.")
    if int(point_index) == 0:
        return center

    index = int(point_index)
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))
    radius_ratio = 0.9 * (index / 99.0) ** 0.5
    height_ratio = 0.9 * (1.0 - 2.0 * (index - 1) / 98.0)
    angle = golden_angle * index
    return rg.Point3d(
        center.X + radius * radius_ratio * math.cos(angle),
        center.Y + radius * radius_ratio * math.sin(angle),
        center.Z + half_height * height_ratio,
    )


def _generate_box_point(center, half_x, half_y, half_z, point_index):
    """Generate one of 100 points inside a box volume."""
    if point_index is None:
        point_index = 0
    # ERROR: point_index must select one of the 100 generated points.
    if (
        isinstance(point_index, bool)
        or int(point_index) != point_index
        or point_index < 0
        or point_index > 99
    ):
        raise ValueError("point_index must be an integer from 0 to 99.")
    if int(point_index) == 0:
        return center

    index = int(point_index)
    golden_ratio = (1.0 + math.sqrt(5.0)) / 2.0
    scale = 0.9
    x_ratio = 2.0 * ((index * golden_ratio) % 1.0) - 1.0
    y_ratio = 2.0 * ((index * golden_ratio ** 2) % 1.0) - 1.0
    z_ratio = 2.0 * ((index * golden_ratio ** 3) % 1.0) - 1.0
    return rg.Point3d(
        center.X + half_x * scale * x_ratio,
        center.Y + half_y * scale * y_ratio,
        center.Z + half_z * scale * z_ratio,
    )


def create_cylinder_shells(
    cylinder_brep,
    cell_size,
    sphere_count,
    offset_cells,
    prism_brep=None,
    vertex_index=0,
    point_index=0,
):
    """Create alternating concentric cylindrical shells from a cylinder Brep."""
    # ERROR: cylinder_brep must be a valid Rhino Brep.
    if cylinder_brep is None or not cylinder_brep.IsValid:
        raise ValueError("cylinder_brep must be a valid Rhino.Geometry.Brep.")
    # ERROR: cell_size must be positive.
    if cell_size <= 0:
        raise ValueError("cell_size must be positive.")
    # ERROR: sphere_count must be an integer of at least 2.
    if (
        isinstance(sphere_count, bool)
        or int(sphere_count) != sphere_count
        or sphere_count < 2
    ):
        raise ValueError("sphere_count must be an integer of at least 2.")
    # ERROR: offset_cells must be a positive integer.
    if (
        isinstance(offset_cells, bool)
        or int(offset_cells) != offset_cells
        or offset_cells < 1
    ):
        raise ValueError("offset_cells must be a positive integer.")

    sphere_count = int(sphere_count)
    offset_distance = int(offset_cells) * cell_size
    bounding_box = cylinder_brep.GetBoundingBox(True)
    center = bounding_box.Center
    radius = max(
        bounding_box.Max.X - center.X,
        bounding_box.Max.Y - center.Y,
        bounding_box.Max.Z - center.Z,
    )
    # ERROR: cylinder_brep must have a positive bounding radius.
    if radius <= 0:
        raise ValueError("cylinder_brep must have a positive bounding radius.")

    cylinders = []
    half_height = (bounding_box.Max.Z - bounding_box.Min.Z) / 2.0
    for index in range(sphere_count):
        cylinder = rg.Cylinder(
            rg.Circle(
                rg.Plane(center, rg.Vector3d.ZAxis),
                radius + index * offset_distance,
            ),
            2.0 * (half_height + index * offset_distance),
        )
        cylinders.append(cylinder.ToBrep(True, True))

    # Generate points inside the complete geometry, including the outermost cylinder.
    generated_point = _generate_cylinder_point(
        center,
        radius + (sphere_count - 1) * offset_distance,
        half_height + (sphere_count - 1) * offset_distance,
        point_index,
    )

    shells = []
    for index in range(0, sphere_count - 1, 2):
        outer = cylinders[index + 1]
        inner = cylinders[index]
        inner.Flip()
        outer.Append(inner)
        # ERROR: Rhino must successfully create each cylindrical shell.
        if not outer.IsValid:
            raise RuntimeError(
                "Rhino could not create cylindrical shell {}.".format(index)
            )
        shells.append(outer)

    shells = _cut_shells_with_prism(
        shells, prism_brep, generated_point, vertex_index, cell_size
    )
    return _add_shell_materials(shells)


def create_cube_shells(
    cube_brep,
    cell_size,
    sphere_count,
    offset_cells,
    prism_brep=None,
    vertex_index=0,
    point_index=0,
):
    """Create alternating concentric cubic shells from a cube Brep."""
    # ERROR: cube_brep must be a valid Rhino Brep.
    if cube_brep is None or not cube_brep.IsValid:
        raise ValueError("cube_brep must be a valid Rhino.Geometry.Brep.")
    # ERROR: cell_size must be positive.
    if cell_size <= 0:
        raise ValueError("cell_size must be positive.")
    # ERROR: sphere_count must be an integer of at least 2.
    if (
        isinstance(sphere_count, bool)
        or int(sphere_count) != sphere_count
        or sphere_count < 2
    ):
        raise ValueError("sphere_count must be an integer of at least 2.")
    # ERROR: offset_cells must be a positive integer.
    if (
        isinstance(offset_cells, bool)
        or int(offset_cells) != offset_cells
        or offset_cells < 1
    ):
        raise ValueError("offset_cells must be a positive integer.")

    sphere_count = int(sphere_count)
    offset_distance = int(offset_cells) * cell_size
    bounding_box = cube_brep.GetBoundingBox(True)
    center = bounding_box.Center
    side_x = bounding_box.Max.X - bounding_box.Min.X
    side_y = bounding_box.Max.Y - bounding_box.Min.Y
    side_z = bounding_box.Max.Z - bounding_box.Min.Z
    # ERROR: cube_brep must have positive bounding-box dimensions.
    if min(side_x, side_y, side_z) <= 0:
        raise ValueError("cube_brep must have positive bounding-box dimensions.")

    cubes = []
    for index in range(sphere_count):
        half_x = side_x / 2.0 + index * offset_distance
        half_y = side_y / 2.0 + index * offset_distance
        half_z = side_z / 2.0 + index * offset_distance
        cubes.append(
            rg.Box(
                rg.Plane(center, rg.Vector3d.XAxis, rg.Vector3d.YAxis),
                rg.Interval(-half_x, half_x),
                rg.Interval(-half_y, half_y),
                rg.Interval(-half_z, half_z),
            ).ToBrep()
        )

    # Generate points inside the complete geometry, including the outermost cube.
    outer_index = sphere_count - 1
    generated_point = _generate_box_point(
        center,
        side_x / 2.0 + outer_index * offset_distance,
        side_y / 2.0 + outer_index * offset_distance,
        side_z / 2.0 + outer_index * offset_distance,
        point_index,
    )

    shells = []
    for index in range(0, sphere_count - 1, 2):
        outer = cubes[index + 1]
        inner = cubes[index]
        inner.Flip()
        outer.Append(inner)
        # ERROR: Rhino must successfully create each cubic shell.
        if not outer.IsValid:
            raise RuntimeError(
                "Rhino could not create cubic shell {}.".format(index)
            )
        shells.append(outer)

    shells = _cut_shells_with_prism(
        shells, prism_brep, generated_point, vertex_index, cell_size
    )
    return _add_shell_materials(shells)


