pub use sea_orm_migration::prelude::*;

mod m20260727_180611_create_grid_worlds;
mod m20260730_213446_create_plans;
mod m20260806_120000_unique_grid_world_name_version;
mod m20260814_201518_add_sim_interval_to_grid_worlds;
mod m20260818_015346_create_robots;
mod m20260818_165852_create_plan_robots;

pub struct Migrator;

#[async_trait::async_trait]
impl MigratorTrait for Migrator {
    fn migrations() -> Vec<Box<dyn MigrationTrait>> {
        vec![
            Box::new(m20260727_180611_create_grid_worlds::Migration),
            Box::new(m20260730_213446_create_plans::Migration),
            Box::new(m20260806_120000_unique_grid_world_name_version::Migration),
            Box::new(m20260814_201518_add_sim_interval_to_grid_worlds::Migration),
            Box::new(m20260818_015346_create_robots::Migration),
            Box::new(m20260818_165852_create_plan_robots::Migration),
        ]
    }
}
