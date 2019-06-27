defmodule ElixirRigidPhysics.Geometry.Sphere do
  @moduledoc """
  Sphere geometry module.

  [Spheres](https://en.wikipedia.org/wiki/Sphere) are solid volumes defined by points within some radius of the origin.

  * `radius` -- the radius of the sphere.

  Their center is assumed to be at the origin, but in the absence of a frame that doesn't matter.
  """
  require Record
  Record.defrecord(:sphere, radius: 1.0)

  @doc """
  Creates a sphere geometry.

  `radius` is the radius of the sphere.

  ## Examples

    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    ElixirRigidPhysics.Geometry.Sphere
    iex> Sphere.create(2.0)
    {:sphere, 2.0}
  """
  def create(radius), do: sphere(radius: radius)
end
