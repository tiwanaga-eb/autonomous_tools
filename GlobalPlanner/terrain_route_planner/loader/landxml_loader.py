"""Load a LandXML TIN surface into vertex and face arrays."""
import numpy as np
import sys


def load_landxml_tin(filepath: str) -> tuple:
    """Parse a LandXML file and return (vertices, faces) arrays for all surfaces."""
    try:
        from lxml import etree
    except ImportError:
        print("lxml not found. Run: pip install lxml", file=sys.stderr)
        sys.exit(1)

    tree = etree.parse(filepath)
    root = tree.getroot()

    # Try namespace versions in order of preference
    ns = None
    for version in ["1.2", "1.1", "1.0"]:
        candidate = {"lx": f"http://www.landxml.org/schema/LandXML-{version}"}
        surfaces = root.findall(".//lx:Surface", candidate)
        if surfaces:
            ns = candidate
            break

    # Fallback: no namespace
    if ns is None:
        surfaces = root.findall(".//Surface")
        if surfaces:
            ns = {}
        else:
            raise ValueError(
                "No TIN surface found in XML. "
                "Check LandXML schema version and `<Surface>` export settings."
            )

    if len(surfaces) > 1:
        print(f"Warning: {len(surfaces)} surfaces found in '{filepath}' — merging all.")

    all_vertices = []
    all_faces = []
    vertex_offset = 0

    for surf in surfaces:
        # Locate <Pnts> and <Faces> blocks
        if ns:
            pnts_el = surf.find(".//lx:Pnts", ns)
            faces_el = surf.find(".//lx:Faces", ns)
        else:
            pnts_el = surf.find(".//Pnts")
            faces_el = surf.find(".//Faces")

        if pnts_el is None:
            raise ValueError(
                "No TIN surface found in XML. "
                "Check LandXML schema version and `<Surface>` export settings."
            )

        # Parse vertices: <P id="...">northing easting elevation</P>
        if ns:
            p_tag = f"{{{ns['lx']}}}P" if ns else "P"
        else:
            p_tag = "P"

        vertex_map = {}  # id -> index
        verts = []
        for p in pnts_el:
            pid = int(p.get("id"))
            parts = p.text.strip().split()
            northing, easting, elevation = float(parts[0]), float(parts[1]), float(parts[2])
            vertex_map[pid] = len(verts)
            verts.append([easting, northing, elevation])  # X=easting, Y=northing, Z=elev

        verts_arr = np.array(verts, dtype=np.float64)

        # Parse faces: <F>1 2 3</F> — 1-based IDs
        faces_list = []
        if faces_el is not None:
            for f in faces_el:
                ids = [int(x) for x in f.text.strip().split()]
                faces_list.append([vertex_map[i] + vertex_offset for i in ids])

        faces_arr = np.array(faces_list, dtype=np.int32) if faces_list else np.zeros((0, 3), dtype=np.int32)

        all_vertices.append(verts_arr)
        all_faces.append(faces_arr)
        vertex_offset += len(verts_arr)

    vertices = np.concatenate(all_vertices, axis=0)
    faces = np.concatenate(all_faces, axis=0)

    print(f"Loaded TIN: {len(vertices):,} vertices, {len(faces):,} faces from {filepath}")
    return vertices, faces
