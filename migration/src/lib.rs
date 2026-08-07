pub use sea_orm_migration::prelude::*;

mod m20260727_180611_create_grid_worlds;
mod m20260730_213446_create_plans;
mod m20260806_120000_unique_grid_world_name_version;

pub struct Migrator;

#[async_trait::async_trait]
impl MigratorTrait for Migrator {
    fn migrations() -> Vec<Box<dyn MigrationTrait>> {
        vec![
            Box::new(m20260727_180611_create_grid_worlds::Migration),
            Box::new(m20260730_213446_create_plans::Migration),
            Box::new(m20260806_120000_unique_grid_world_name_version::Migration),
        ]
    }
}
