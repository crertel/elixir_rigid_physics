defmodule ElixirRigidPhysics.Geometry.LineSegment do

  @moduledoc """
  Line segment geometry module.

  Line segemnts go from a start point `a` to an end point `b`.
  """
  require Record
  Record.defrecord(:line_segment, a: {0.0, 0.0, 0.0}, b: {0.0, 0.0, 0.0})
  @type line_segment :: record(:line_segment, a: Vec3.vec3, b: Vec3.vec3)
  alias Graphmath.Vec3

  @doc """
  Creates a line segment.

  ## Examples
    iex> # IO.puts("Test line segment creation.")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> LineSegment.create( {0.0, 1.0, 0.0}, {0.0, 1.0, 1.0} )
    {:line_segment, {0.0, 1.0, 0.0}, {0.0, 1.0, 1.0} }
  """
  @spec create(Vec3.vec3, Vec3.vec3) :: line_segment
  def create(a, b), do: line_segment(a: a, b: b)


  @doc """
  Projects a point `p` onto the same line as line segment `ab`. **Note that the point may not be on the segment itself.**

  ## Examples
    iex> #IO.puts("Check segment coincident with a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> p = {0.0, 0.0, 0.0}
    iex> LineSegment.project( segment, p)
    {0.0, 0.0, 0.0}

    iex> #IO.puts("Check segment coincident with b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> p = {1.0, 0.0, 0.0}
    iex> LineSegment.project( segment, p)
    {1.0, 0.0, 0.0}

    iex> #IO.puts("Check segment interior of segment")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> p = {0.5, 0.0, 0.0}
    iex> LineSegment.project( segment, p)
    {0.5, 0.0, 0.0}

    iex> #IO.puts("Check segment in voronoi region of a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> p = {-0.5, 0.0, 0.0}
    iex> LineSegment.project( segment, p)
    {-0.5, 0.0, 0.0}

    iex> #IO.puts("Check segment in voronoi region of b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> p = {1.5, 0.0, 0.0}
    iex> LineSegment.project( segment, p)
    {1.5, 0.0, 0.0}

    iex> #IO.puts("Check offset segment in voronoi region of a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> p = {-0.5, 3.0, 4.0}
    iex> LineSegment.project( segment, p)
    {-0.5, 0.0, 0.0}

    iex> #IO.puts("Check offset segment in voronoi region of b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> p = {1.5, 3.0, 4.0}
    iex> LineSegment.project( segment, p)
    {1.5, 0.0, 0.0}

    iex> #IO.puts("Check offset segment in voronoi region of segment")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> p = {0.45, 3.0, 4.0}
    iex> LineSegment.project( segment, p)
    {0.45, 0.0, 0.0}

  """
  @spec project( line_segment, Vec3.vec3) :: Vec3.vec3
  def project( line_segment(a: a, b: b),  p) do
    dir = Vec3.subtract(b, a) |> Vec3.normalize()
    Vec3.add(a, Vec3.scale( dir, Vec3.dot(p, dir)))
  end

  @doc """
  Gets the [Barycentric coordinates](https://en.wikipedia.org/wiki/Barycentric_coordinate_system) of a point on the segment.
  """
  @spec barycentric( line_segment, Vec3.vec3) :: {float, float}
  def barycentric( segment, {x,y,z}) do
    {0.0, 0.0}
  end
end
