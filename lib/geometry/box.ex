defmodule ElixirRigidPhysics.Geometry.Box do
  @moduledoc """
  Box geometry module.

  Boxes are solid rectangular prisms, with three dimensions:

  * `width` -- length on the x-axis
  * `height` -- length on the y-axis (up)
  * `depth` -- length on the z-axis

  Their center is assumed to be at the origin, but in the absence of a frame that doesn't matter.
  """
  require Record
  Record.defrecord(:box, width: 0.0, height: 0.0, depth: 0.0)
  @type box :: record(:box, width: number, height: number, depth: number)

  @doc """
  Creates a box geometry.

  Width is the length on the x-axis, height is the length on the y-axis, and depth is the length on the z-axis.

  ## Examples

    iex> require ElixirRigidPhysics.Geometry.Box, as: Box
    ElixirRigidPhysics.Geometry.Box
    iex> Box.create(1.0,2.0,3.0)
    {:box, 1.0, 2.0, 3.0}
  """
  @spec create(number, number, number) :: box
  def create(w, h, d), do: box(width: w, height: h, depth: d)
end
