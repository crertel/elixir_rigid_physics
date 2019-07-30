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



end
