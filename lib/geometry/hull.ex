defmodule ElixirRigidPhysics.Geometry.Hull do
  @moduledoc """
  Hull geometry module.

  Hulls are lists of triangular faces wound CCW (to find normal, follow right-hand rule).

  Their center is assumed to be at the origin, but in the absence of a frame that doesn't matter.
  """
  alias Graphmath.Vec3

  require Record
  Record.defrecord(:hull, faces: [])
  @type hull_face :: { Vec3.vec3, Vec3.vec3, Vec3.vec3 }
  @type hull :: record(:hull, faces: [hull_face])

  @doc """
  Creates a hull geometry.

  Given a list of faces (wound CCW), creates a hull geometry.

  ## Examples

    iex> require ElixirRigidPhysics.Geometry.Box, as: Box
    ElixirRigidPhysics.Geometry.Box
    iex> Box.create(1.0,2.0,3.0)
    {:box, 1.0, 2.0, 3.0}
  """
  @spec create([hull_face]) :: hull
  def create(faces), do: hull(faces: faces)
end
