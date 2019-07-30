defmodule ElixirRigidPhysics.Geometry.Tetrahedron do
  @moduledoc """
  Module for handling queries related to tetrahedra.
  """
  alias Graphmath.Vec3

  require Record
  Record.defrecord(:tetrahedron, a: {0.0, 0.0, 0.0}, b: {1.0, 0.0, 0.0}, c: {0.0, 1.0, 0.0}, d: {0.0, 0.0, 1.0})
  @type tetrahedron :: record(:tetrahedron, a: Vec3.vec3(), b: Vec3.vec3(), c: Vec3.vec3(), d: Vec3.vec3())

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
    v_qbcd = create(q,b,c,d) |> volume() # a
    v_qacd = create(q,c,a,d) |> volume() # b
    v_qabd = create(q,a,b,d) |> volume() # c
    v_qcab = create(q,a,c,b) |> volume() # d
    v_total = volume(t)

    { v_qbcd / v_total, v_qacd / v_total, v_qabd / v_total, v_qcab / v_total}
  end

end
