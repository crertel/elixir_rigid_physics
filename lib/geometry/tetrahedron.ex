defmodule ElixirRigidPhysics.Geometry.Tetrahedron do
  @moduledoc """
  Module for handling queries related to tetrahedra.
  """
  alias Graphmath.Vec3

  require Record
  Record.defrecord(:tetrahedron, a: {0.0, 0.0, 0.0}, b: {1.0, 0.0, 0.0}, c: {0.0, 1.0, 0.0}, d: {0.0, 0.0, 1.0})
  @type tetrahedron :: record(:tetrahedron, a: Vec3.vec3, b: Vec3.vec3, c: Vec3.vec3, d: Vec3.vec3)

  require ElixirRigidPhysics.Geometry.LineSegment, as: LSeg
  require ElixirRigidPhysics.Geometry.Triangle, as: Tri
  require ElixirRigidPhysics.Geometry.Plane, as: Plane

  @verysmol 1.0e-12

  @doc """
  Function to create a tetrahedron given four points.

  ## Examples
    iex> # IO.puts "Check creating a tetrahedron."
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> Tetra.create(a,b,c,d)
    {:tetrahedron, {1.0, 1.0, 1.0}, {2.0, 1.0, 1.0}, {1.0, 2.0, 1.0}, {1.0, 1.0, 2.0}}
  """
  @spec create( Vec3.vec3, Vec3.vec3, Vec3.vec3, Vec3.vec3) :: tetrahedron
  def create(a,b,c,d) do
    tetrahedron(a: a, b: b, c: c, d: d)
  end

  @doc """
  Calculates the volume of a tetrahedron.
    iex> # IO.puts "Check creating a tetrahedron."
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> 0.0001 > abs(Tetra.volume(t) - 0.16666666)
    true
  """
  @spec volume(tetrahedron) :: float()
  def volume(tetrahedron(a: a, b: b, c: c, d: d)) do
    v_bc = Vec3.subtract(b, c)
    v_dc = Vec3.subtract(d, c)
    v_ac = Vec3.subtract(a, c)
    (1.0 / 6.0) * Vec3.scalar_triple(v_bc, v_dc, v_ac)
  end

  @doc """
  Calculates the Barycentric coordinates in tetrahedron `t` for a query point `p`.

  ## Examples
    iex> # IO.puts "Check query point coincident with a."
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> Tetra.to_barycentric(t, a)
    {1.0, 0.0, 0.0, 0.0}

    iex> # IO.puts "Check query point coincident with b."
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> Tetra.to_barycentric(t, b)
    {0.0, 1.0, 0.0, 0.0}

    iex> # IO.puts "Check query point coincident with c."
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> Tetra.to_barycentric(t, c)
    {0.0, 0.0, 1.0, 0.0}

    iex> # IO.puts "Check query point coincident with d."
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> Tetra.to_barycentric(t, d)
    {0.0, 0.0, 0.0, 1.0}

    iex> # IO.puts "Check query point outside face BCD."
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> Tetra.to_barycentric(t, {2.0, 2.0, 2.0})
    {-2.0, 1.0, 1.0, 1.0}
  """
  @spec to_barycentric(tetrahedron, Vec3.vec3) :: { float, float, float, float}
  def to_barycentric( tetrahedron(a: a, b: b, c: c, d: d) = t, q ) do
    # fun note...with an apex of q, each face must be wound the same way
    v_qbcd = create(q,b,c,d) |> volume() # a
    v_qacd = create(q,c,a,d) |> volume() # b
    v_qabd = create(q,a,b,d) |> volume() # c
    v_qcab = create(q,a,c,b) |> volume() # d
    v_total = volume(t)

    { v_qbcd / v_total, v_qacd / v_total, v_qabd / v_total, v_qcab / v_total}
  end

  @doc """
  Converts from barycentric coords on a tetrahedron to global cartesian coordinates.

  ## Examples
    iex> # IO.puts "Check barycentric coords for a"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> Tetra.from_barycentric(t, {1.0, 0.0, 0.0, 0.0})
    {1.0, 1.0, 1.0}

    iex> # IO.puts "Check barycentric coords for b"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> Tetra.from_barycentric(t, {0.0, 1.0, 0.0, 0.0})
    {2.0, 1.0, 1.0}

    iex> # IO.puts "Check barycentric coords for c"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> Tetra.from_barycentric(t, {0.0, 0.0, 1.0, 0.0})
    {1.0, 2.0, 1.0}

    iex> # IO.puts "Check barycentric coords for d"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> a = {1.0, 1.0, 1.0}
    iex> b = {2.0, 1.0, 1.0}
    iex> c = {1.0, 2.0, 1.0}
    iex> d = {1.0, 1.0, 2.0}
    iex> t = Tetra.create(a,b,c,d)
    iex> Tetra.from_barycentric(t, {0.0, 0.0, 0.0, 1.0})
    {1.0, 1.0, 2.0}

  """
  @spec from_barycentric(tetrahedron, {number, number, number, number}) :: Vec3.vec3
  def from_barycentric( tetrahedron(a: a, b: b, c: c, d: d), {b_a,b_b,b_c,b_d}) do
    Vec3.add( Vec3.weighted_sum(b_a, a, b_b, b), Vec3.weighted_sum(b_c, c, b_d, d))
  end

  @type voronoi_vertex_region :: :region_a | :region_b | :region_c | :region_d
  @type voronoi_edge_region :: :region_ab | :region_ac | :region_ad | :region_bc | :region_bd | :region_cd
  @type voronoi_face_region :: :region_abc | :region_abd | :region_acd | :region_bcd
  @type voronoi_polyhedron_region :: :region_abcd
  @type voronoi_region :: voronoi_vertex_region | voronoi_edge_region | voronoi_face_region | voronoi_polyhedron_region

  @doc """
  Find the nearest point on or in a tetrahedron to a query point, and also return its voronoi region.

  ## Examples
    iex> #IO.puts "Classify internal voronoi region abcd"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.1, 0.1, 0.1}
    iex> Tetra.get_nearest_point( tetra, q)
    {{0.1, 0.1, 0.1}, :region_abcd}

    iex> #IO.puts "Classify vertex voronoi region a, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.0, 0.0, 0.0}
    iex> Tetra.get_nearest_point( tetra, q)
    { {0.0, 0.0, 0.0}, :region_a}

    iex> #IO.puts "Classify vertex voronoi region a, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {-1.0, -1.0, -1.0}
    iex> Tetra.get_nearest_point( tetra, q)
    { {0.0, 0.0, 0.0}, :region_a}

    iex> #IO.puts "Classify vertex voronoi region b, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.0, 0.0, 1.0}
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.0, 0.0, 1.0}, :region_b}

    iex> #IO.puts "Classify vertex voronoi region b, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {-0.1, -0.1, 1.5}
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.0, 0.0, 1.0}, :region_b}

    iex> #IO.puts "Classify vertex voronoi region c, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {1.0, 0.0, 0.0}
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 1.0, 0.0, 0.0}, :region_c}

    iex> #IO.puts "Classify vertex voronoi region c, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {1.5, -0.1, -0.1 }
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 1.0, 0.0, 0.0}, :region_c}

    iex> #IO.puts "Classify vertex voronoi region d, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.0, 1.0, 0.0}
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.0, 1.0, 0.0}, :region_d}

    iex> #IO.puts "Classify vertex voronoi region d, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {-0.1, 1.5, -0.1 }
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.0, 1.0, 0.0}, :region_d}

    iex> #IO.puts "Classify edge voronoi region ab, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.0, 0.0, 0.5}
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.0, 0.0, 0.5}, :region_ab}

    iex> #IO.puts "Classify edge voronoi region ab, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {-0.1, -0.1, 0.5 }
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.0, 0.0, 0.5}, :region_ab}

    iex> #IO.puts "Classify edge voronoi region ac, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.5, 0.0, 0.0}
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.5, 0.0, 0.0}, :region_ac}

    iex> #IO.puts "Classify edge voronoi region ac, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.5, -0.1, -0.1 }
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.5, 0.0, 0.0}, :region_ac}

    iex> #IO.puts "Classify edge voronoi region ad, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.0, 0.5, 0.0}
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.0, 0.5, 0.0}, :region_ad}

    iex> #IO.puts "Classify edge voronoi region ad, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {-0.1, 0.5, -0.1 }
    iex> Tetra.get_nearest_point( tetra, q)
    {{ 0.0, 0.5, 0.0}, :region_ad}

    iex> #IO.puts "Classify edge voronoi region bc, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.5, 0.0, 0.5}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> { Graphmath.Vec3.equal( nearest_point, {0.5, 0.0, 0.5}, 0.0001), region}
    {true, :region_bc}

    iex> #IO.puts "Classify edge voronoi region bc, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.5, -0.1, 0.5 }
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> { Graphmath.Vec3.equal( nearest_point, {0.5, 0.0, 0.5}, 0.0001), region}
    {true, :region_bc}

    iex> #IO.puts "Classify edge voronoi region bd, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.0, 0.5, 0.5}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> { Graphmath.Vec3.equal( nearest_point, {0.0, 0.5, 0.5}, 0.0001), region}
    {true, :region_bd}

    iex> #IO.puts "Classify edge voronoi region bd, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {-0.1, 0.5, 0.5 }
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> { Graphmath.Vec3.equal( nearest_point, {0.0, 0.5, 0.5}, 0.0001), region}
    {true, :region_bd}

    iex> #IO.puts "Classify edge voronoi region cd, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.5, 0.5, 0.0}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {0.5, 0.5, 0.0}, 0.0001), region}
    {true, :region_cd}

    iex> #IO.puts "Classify edge voronoi region cd, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0.5, 0.5, -1.0 }
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {0.5, 0.5, 0.0}, 0.0001), region}
    {true, :region_cd}

    iex> #IO.puts "Classify face voronoi region abc, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {1/3, 0, 1/3}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {1/3, 0, 1/3}, 0.0001), region}
    {true, :region_abc}

    iex> #IO.puts "Classify face voronoi region abc, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {1/3, -1, 1/3}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {1/3, 0, 1/3}, 0.0001), region}
    {true, :region_abc}

    iex> #IO.puts "Classify face voronoi region abd, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {0, 1/3, 1/3}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {0, 1/3, 1/3}, 0.0001), region}
    {true, :region_abd}

    iex> #IO.puts "Classify face voronoi region abd, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {-1, 1/3, 1/3}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {0, 1/3, 1/3}, 0.0001), region}
    {true, :region_abd}

    iex> #IO.puts "Classify face voronoi region bcd, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {1/3, 1/3, 1/3}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {1/3, 1/3, 1/3}, 0.0001), region}
    {true, :region_bcd}

    iex> #IO.puts "Classify face voronoi region bcd, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {4/3, 4/3, 4/3}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {1/3, 1/3, 1/3}, 0.0001), region}
    {true, :region_bcd}

    iex> #IO.puts "Classify face voronoi region cad, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {1/3, 1/3, 0}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {1/3, 1/3, 0.0}, 0.0001), region}
    {true, :region_cad}

    iex> #IO.puts "Classify face voronoi region cad, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> q = {1/3, 1/3, -2}
    iex> {nearest_point, region} = Tetra.get_nearest_point( tetra, q)
    iex> {Graphmath.Vec3.equal( nearest_point, {1/3, 1/3, 0.0}, 0.0001), region}
    {true, :region_cad}
  """
  @spec get_nearest_point( tetrahedron, Vec3.vec3) :: { Vec3.vec3, voronoi_region}
  def get_nearest_point( tetrahedron(a: a, b: b, c: c, d: d) = tetra, q ) do
    # I hope you like barycenters, 'cause we're about to calculate a lot of them.
    { qa, qb, qc, qd } = to_barycentric(tetra, q)

    tri_acb = Tri.create_from_points(a,c,b)
    acb_plane = Tri.to_plane(tri_acb)
    acb_q = Plane.project_point_to_plane(acb_plane, q)
    { tri_acb_u, tri_acb_v, tri_acb_w} = Tri.to_barycentric(tri_acb, acb_q)

    tri_abd = Tri.create_from_points(a,b,d)
    abd_plane = Tri.to_plane(tri_abd)
    abd_q = Plane.project_point_to_plane(abd_plane, q)
    { tri_abd_u, tri_abd_v, tri_abd_w} = Tri.to_barycentric(tri_abd, abd_q)

    tri_bcd = Tri.create_from_points(b,c,d)
    bcd_plane = Tri.to_plane(tri_bcd)
    bcd_q = Plane.project_point_to_plane(bcd_plane, q)
    { tri_bcd_u, tri_bcd_v, tri_bcd_w} = Tri.to_barycentric(tri_bcd, bcd_q)

    tri_cad = Tri.create_from_points(c,a,d)
    cad_plane = Tri.to_plane(tri_cad)
    cad_q = Plane.project_point_to_plane(cad_plane, q)
    { tri_cad_u, tri_cad_v, tri_cad_w} = Tri.to_barycentric(tri_cad, cad_q)

    l_ab = LSeg.create(a,b)
    {l_ab_u, l_ab_v}  = LSeg.to_barycentric(l_ab, q)
    l_ac = LSeg.create(a,c)
    {l_ac_u, l_ac_v}  = LSeg.to_barycentric(l_ac, q)
    l_ad = LSeg.create(a,d)
    {l_ad_u, l_ad_v}  = LSeg.to_barycentric(l_ad, q)
    l_bc = LSeg.create(b,c)
    {l_bc_u, l_bc_v}  = LSeg.to_barycentric(l_bc, q)
    l_bd = LSeg.create(b,d)
    {l_bd_u, l_bd_v}  = LSeg.to_barycentric(l_bd, q)
    l_cd = LSeg.create(c,d)
    {l_cd_u, l_cd_v}  = LSeg.to_barycentric(l_cd, q)

    # okay, remember:
    # test verts, then edges, then faces...lowest dimension first!
    cond do
      # a
      l_ab_v <= @verysmol and l_ac_v <= @verysmol and l_ad_v <= @verysmol -> {a, :region_a}
      # b
      l_ab_u <= @verysmol and l_bc_v <= @verysmol and l_bd_v <= @verysmol -> {b, :region_b}
      # c
      l_ac_u <= @verysmol and l_bc_u <= @verysmol and l_cd_v <= @verysmol -> {c, :region_c}
      # d
      l_ad_u <= @verysmol and l_bd_u <= @verysmol and l_cd_u <= @verysmol -> {d, :region_d}

      # ab
      l_ab_u > @verysmol and l_ab_v > @verysmol and tri_abd_w <= @verysmol and tri_acb_v <= @verysmol -> { LSeg.from_barycentric( l_ab, {l_ab_u, l_ab_v}), :region_ab}
      # # ac
      l_ac_u > @verysmol and l_ac_v > @verysmol and tri_acb_w <= @verysmol and tri_cad_w <= @verysmol -> { LSeg.from_barycentric( l_ac, {l_ac_u, l_ac_v}), :region_ac}
      # # ad
      l_ad_u > @verysmol and l_ad_v > @verysmol and tri_abd_v <= @verysmol and tri_cad_u <= @verysmol -> { LSeg.from_barycentric(l_ad, {l_ad_u, l_ad_v}), :region_ad}
      # bc
      l_bc_u > @verysmol and l_bc_v > @verysmol and tri_acb_u <= @verysmol and tri_bcd_w <= @verysmol -> { LSeg.from_barycentric(l_bc,{l_bc_u, l_bc_v}), :region_bc}
      # bd
      l_bd_u > @verysmol and l_bd_v > @verysmol and tri_abd_u <= @verysmol and tri_bcd_v <= @verysmol -> { LSeg.from_barycentric(l_bd,{l_bd_u, l_bd_v}), :region_bd}
      # cd
      l_cd_u > @verysmol and l_cd_v > @verysmol and tri_cad_v <= @verysmol and tri_bcd_u <= @verysmol -> { LSeg.from_barycentric(l_cd, {l_cd_u, l_cd_v}), :region_cd}

      # abc
      tri_acb_u > @verysmol and tri_acb_v > @verysmol and tri_acb_w > @verysmol and qd <= @verysmol ->
        {Tri.from_barycentric(tri_acb, {tri_acb_u, tri_acb_v, tri_acb_w}), :region_abc}

      # abd
      tri_abd_u > @verysmol and tri_abd_v > @verysmol and tri_abd_w > @verysmol and qc <= @verysmol ->
        {Tri.from_barycentric(tri_abd, {tri_abd_u, tri_abd_v, tri_abd_w}), :region_abd}

      # bcd
      tri_bcd_u > @verysmol and tri_bcd_v > @verysmol and tri_bcd_w > @verysmol and qa <= @verysmol ->
        {Tri.from_barycentric(tri_bcd, {tri_bcd_u, tri_bcd_v, tri_bcd_w}), :region_bcd}

      # cad
      tri_cad_u > @verysmol and tri_cad_v > @verysmol and tri_cad_w > @verysmol and qb <= @verysmol ->
        {Tri.from_barycentric(tri_cad, {tri_cad_u, tri_cad_v, tri_cad_w}), :region_cad}

      # abcd
      qa > 0 and qb > 0 and qc > 0 and qd > 0 -> {q, :region_abcd}

      true -> IO.puts """
              Something's wrong.

              Q:    #{inspect q}
              AB:   #{inspect {l_ab_u, l_ab_v}}
              AC:   #{inspect {l_ac_u, l_ac_v}}
              AD:   #{inspect {l_ad_u, l_ad_v}}
              BC:   #{inspect {l_bc_u, l_bc_v}}
              BD:   #{inspect {l_bd_u, l_bd_v}}
              CD:   #{inspect {l_cd_u, l_cd_v}}
              ACB:  #{inspect { tri_acb_u, tri_acb_v, tri_acb_w}}
              ABD:  #{inspect { tri_abd_u, tri_abd_v, tri_abd_w}}
              BCD:  #{inspect { tri_bcd_u, tri_bcd_v, tri_bcd_w}}
              CAD:  #{inspect { tri_cad_u, tri_cad_v, tri_cad_w}}
              ABCD: #{inspect { qa, qb, qc, qd }}
              """
    end
  end

  # iex> #IO.puts "Classify face voronoi region abd, near"
  #   iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
  #   iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
  #   iex> Tetra.classify_point_for_tetrahedron( tetra, {-0.0, 0.1, 0.1})
  #   :region_abd

  #   iex> #IO.puts "Classify face voronoi region abd, far"
  #   iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
  #   iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
  #   iex> Tetra.classify_point_for_tetrahedron( tetra, {-2.0, 0.1, 0.1})
  #   :region_abd

  #   iex> #IO.puts "Classify face voronoi region bcd, near"
  #   iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
  #   iex> sqrtthree = :math.sqrt(3.0)
  #   iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
  #   iex> Tetra.classify_point_for_tetrahedron( tetra, {sqrtthree,sqrtthree,sqrtthree})
  #   :region_bcd

  #   iex> #IO.puts "Classify face voronoi region bcd, far"
  #   iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
  #   iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
  #   iex> Tetra.classify_point_for_tetrahedron( tetra, {1.0, 1.0, 1.0})
  #   :region_bcd
end
