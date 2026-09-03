pub use sea_orm_migration::prelude::*;

mod m20260727_180611_create_grid_worlds;
mod m20260818_015346_create_robots;
mod m20260901_171859_create_grid_world_states;
mod m20260901_192414_create_route_plans;
mod m20260902_200304_create_route_histories;

pub struct Migrator;

#[async_trait::async_trait]
impl MigratorTrait for Migrator {
    fn migrations() -> Vec<Box<dyn MigrationTrait>> {
        vec![
            Box::new(m20260727_180611_create_grid_worlds::Migration),
            Box::new(m20260818_015346_create_robots::Migration),
            Box::new(m20260901_171859_create_grid_world_states::Migration),
            Box::new(m20260901_192414_create_route_plans::Migration),
            Box::new(m20260902_200304_create_route_histories::Migration),
        ]
    }
}
