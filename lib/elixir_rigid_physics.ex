defmodule ElixirRigidPhysics do
  @moduledoc """
  Documentation for ElixirRigidPhysics.
  """

  use GenServer
  alias ElixirRigidPhysics.World

  #######################
  ## Client

  def start_link(%World{} = world) do
    GenServer.start_link(__MODULE__, world)
  end

  def get_world_state(pid) do
    GenServer.call(pid, :get_world_state)
  end

  @doc """
  Steps the world simulation.

  Valid options are:

  * `timestep` is the number of seconds to step the world by.
  """
  @spec step_world(pid(), Keyword.t) :: :ok
  def step_world(pid, opts \\ []) do
    GenServer.call(pid, {:step_world, opts})
  end

  def add_body_to_world(pid, body) do
    GenServer.call(pid, {:add_body_to_world, body})
  end

  #################################
  ## Server

  @impl true
  def init(%World{} = world) do
    {:ok, world}
  end

  @impl true
  def handle_call(:get_world_state, _from, %World{} = world) do
    {:reply, world, world}
  end

  @impl true
  def handle_call({:step_world, _opts}, _from, %World{} = world) do
    new_world = world
    {:reply, new_world, new_world}
  end

  @impl true
  def handle_call({:add_body_to_world, body}, _from, %World{bodies: bodies} = world) do
    body_ref = make_ref()
    new_world = %World{world| bodies: Map.put(bodies, body_ref, body)}
    {:reply, new_world, new_world}
  end
end
