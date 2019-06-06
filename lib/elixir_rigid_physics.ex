defmodule ElixirRigidPhysics do
  alias ElixirRigidPhysics.World

  @moduledoc """
  Documentation for ElixirRigidPhysics.
  """

  @doc """
  Creates a world to do simulation on, returning a handle to the world.
  """
  @spec create_world() :: World.t
  def create_world() do
    %World{}
  end

  @spec destroy_world(World.t) :: :ok
  def destroy_world(_world) do
    :ok
  end

  @doc """
  Steps the world simulation.

  Valid options are:

  * `timestep` is the number of seconds to step the world by.
  """
  @spec step_world(World.t, Keyword.t) :: :ok
  def step_world(_world, _opts \\ []) do
    :ok
  end
end
