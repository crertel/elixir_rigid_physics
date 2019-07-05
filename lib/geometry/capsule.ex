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
  @spec get_principle_points( capsule ) :: { {number, number, number}, {number, number, number}}
  def get_principle_points( capsule(axial_length: l)) do
    {
      {0.0, -l / 2.0, 0.0},
      {0.0, l / 2.0, 0.0},
    }
  end
end
