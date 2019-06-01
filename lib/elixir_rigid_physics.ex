defmodule ElixirRigidPhysics do
  @moduledoc """
  Documentation for ElixirRigidPhysics.
  """

  @doc """
  Creates a world to do simulation on, returning a handle to the world.
  """
  def create_world() do
    :world
  end

  @doc """
  Steps the world simulation.
  """
  def step_world(world, opts \\ []) do
    :stepped_world
  end
end
