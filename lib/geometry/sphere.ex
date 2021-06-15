defmodule ElixirRigidPhysics.Geometry.Sphere do
  @moduledoc """
  Sphere geometry module.

  [Spheres](https://en.wikipedia.org/wiki/Sphere) are solid volumes defined by points within some radius of the origin.

  * `radius` -- the radius of the sphere.

  Their center is assumed to be at the origin, but in the absence of a frame that doesn't matter.
  """
  require Record
  Record.defrecord(:sphere, radius: 1.0)
  @type sphere :: record(:sphere, radius: number)

  alias Graphmath.Vec3

  @doc """
  Creates a sphere geometry.

  `radius` is the radius of the sphere.

  ## Examples

    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    ElixirRigidPhysics.Geometry.Sphere
    iex> Sphere.create(2.0)
    {:sphere, 2.0}
  """
  @spec create(number) :: sphere
  def create(radius), do: sphere(radius: radius)

  @doc """
  Finds a support point (usually for GJK) on a sphere given a search direction.

  ## Examples
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> s = Sphere.create(2.0)
    iex> Sphere.support_point(s, {1.0,0.0,0.0})
    {2.0, 0.0, 0.0}
  """
  @spec support_point(sphere, Vec3.vec3()) :: Vec3.vec3()
  def support_point(sphere(radius: r), direction) do
    Vec3.scale(direction, r)
  end
end
