defmodule ElixirRigidPhysics.Geometry.Tetrahedron do
  @moduledoc """
  Module for handling queries related to tetrahedra.
  """
  alias Graphmath.Vec3

  require Record
  Record.defrecord(:tetrahedron, a: {0.0, 0.0, 0.0}, b: {1.0, 0.0, 0.0}, c: {0.0, 1.0, 0.0}, d: {0.0, 0.0, 1.0})
  @type tetrahedron :: record(:tetrahedron, a: Vec3.vec3, b: Vec3.vec3, c: Vec3.vec3, d: Vec3.vec3)

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
  Figure out which voronoi region of a tetrahedron a point is in.

  ## Examples
    iex> #IO.puts "Classify internal voronoi region abcd"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {0.1, 0.1, 0.1})
    :region_abcd

    iex> #IO.puts "Classify vertex voronoi region a, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {0.0, 0.0, 0.0})
    :region_a

    iex> #IO.puts "Classify vertex voronoi region a, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {-1.0, -1.0, -1.0})
    :region_a

    iex> #IO.puts "Classify vertex voronoi region b, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {0.0, 0.0, 1.0})
    :region_b

    iex> #IO.puts "Classify vertex voronoi region b, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {-0.1, -0.1, 1.5})
    :region_b

    iex> #IO.puts "Classify vertex voronoi region c, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {1.0, 0.0, 0.0})
    :region_c

    iex> #IO.puts "Classify vertex voronoi region c, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {1.5, -0.1, -0.1})
    :region_c

    iex> #IO.puts "Classify vertex voronoi region d, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {0.0, 1.0, 0.0})
    :region_d

    iex> #IO.puts "Classify vertex voronoi region d, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {-0.1, 1.5, -0.1})
    :region_d

    iex> #IO.puts "Classify face voronoi region abd, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {-0.0, 0.1, 0.1})
    :region_abd

    iex> #IO.puts "Classify face voronoi region abd, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {-2.0, 0.1, 0.1})
    :region_abd

    iex> #IO.puts "Classify face voronoi region bcd, near"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> sqrtthree = :math.sqrt(3.0)
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {sqrtthree,sqrtthree,sqrtthree})
    :region_bcd

    iex> #IO.puts "Classify face voronoi region bcd, far"
    iex> require ElixirRigidPhysics.Geometry.Tetrahedron, as: Tetra
    iex> tetra = Tetra.create( {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
    iex> Tetra.classify_point_for_tetrahedron( tetra, {1.0, 1.0, 1.0})
    :region_bcd
  """
  @spec classify_point_for_tetrahedron( tetrahedron, Vec3.vec3) :: voronoi_region
  def classify_point_for_tetrahedron( tetra, q ) do
    { a, b, c, d } = to_barycentric(tetra, q)
    cond do
      a >= 0 and b <= 0 and c <= 0 and d <= 0 -> :region_a
      a <= 0 and b >= 0 and c <= 0 and d <= 0 -> :region_b
      a <= 0 and b <= 0 and c >= 0 and d <= 0 -> :region_c
      a <= 0 and b <= 0 and c <= 0 and d >= 0 -> :region_d

      a >= 0 and b >= 0 and c <= 0 and d <= 0 -> :region_ab
      a >= 0 and b <= 0 and c >= 0 and d <= 0 -> :region_ac
      a >= 0 and b <= 0 and c <= 0 and d >= 0 -> :region_ad
      a <= 0 and b >= 0 and c >= 0 and d <= 0 -> :region_bc
      a <= 0 and b >= 0 and c <= 0 and d >= 0 -> :region_bd
      a <= 0 and b <= 0 and c >= 0 and d >= 0 -> :region_cd

      a >= 0 and b >= 0 and c >= 0 and d <= 0 -> :region_abc
      a >= 0 and b >= 0 and c <= 0 and d >= 0 -> :region_abd
      a >= 0 and b <= 0 and c >= 0 and d >= 0 -> :region_acd
      a <= 0 and b >= 0 and c >= 0 and d >= 0 -> :region_bcd

      a > 0 and b > 0 and c >  0 and d > 0 -> :region_abcd
    end
  end
end
