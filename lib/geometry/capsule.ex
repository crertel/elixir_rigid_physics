defmodule ElixirRigidPhysics.Geometry.Capsule do
  @moduledoc """
  Capsule geometry module.

  [Capsules](https://en.wikipedia.org/wiki/Capsule_(geometry)) are cylinders capped with hemispheres, with two dimensions:

  * `axial_length` -- the length of the cylinder (not including hemispheres)
  * `cap_radius` -- the radius of the hemispheres

  Their center is assumed to be at the origin, but in the absence of a frame that doesn't matter.

  They are assumed to be vertically-oriented; their main axis is aligned with the y-axis (up).
  """
  require Record
  Record.defrecord(:capsule, axial_length: 0.5, cap_radius: 0.25)
  @type capsule :: record(:capsule, axial_length: number, cap_radius: number)

  alias Graphmath.Vec3

  @doc """
  Creates a capsule geometry.

  `axial_length` is its length on the y-axis (up) without the hemispheres, `cap_radius` is the radius of the hemisphere ends.

  ## Examples

    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    ElixirRigidPhysics.Geometry.Capsule
    iex> Capsule.create(2.0, 3.0)
    {:capsule, 2.0, 3.0}
  """
  @spec create(number, number) :: capsule
  def create(axial_length, cap_radius),
    do: capsule(axial_length: axial_length, cap_radius: cap_radius)

  @doc """
  Gets the interior principle points of a capsule in local space.

  "principle points" are the point on the capsule at the center of where the axis starts capping.

  ## Examples
    iex> # test unit capsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> Capsule.get_principle_points( Capsule.capsule(axial_length: 1.0, cap_radius: 0.5))
    { {0.0, -0.5, 0.0}, {0.0, 0.5, 0.0} }
    iex> Capsule.get_principle_points( Capsule.capsule(axial_length: 10.0, cap_radius: 0.5))
    { {0.0, -5.0, 0.0}, {0.0, 5.0, 0.0} }
  """
  @spec get_principle_points(capsule) :: {{number, number, number}, {number, number, number}}
  def get_principle_points(capsule(axial_length: l)) do
    {
      {0.0, -l / 2.0, 0.0},
      {0.0, l / 2.0, 0.0}
    }
  end

  @doc """
  Finds a support point (usually for GJK) on a capsule given a search direction.

  ## Examples
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> c = Capsule.create(2.0, 1.0)
    iex> Capsule.support_point(c, {1.0,1.0,1.0})
    {0.0, 1.0, 0.0}

    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> c = Capsule.create(2.0, 1.0)
    iex> Capsule.support_point(c, {1.0,-1.0,1.0})
    {0.0, -1.0, 0.0}
  """
  @spec support_point(capsule, Vec3.vec3()) :: Vec3.vec3()
  def support_point(capsule(axial_length: l), {_x, y, _z} = _direction) do
    # technique borrowed from ReactPhysics3D
    # it weirds me out that the most extreme point is always a capsule end, but maybe that's okay?

    hl = l / 2.0

    dot_top = hl * y
    dot_bottom = -hl * y

    if dot_top > dot_bottom do
      {0.0, hl, 0.0}
    else
      {0.0, -hl, 0.0}
    end
  end
end
