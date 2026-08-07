use sea_orm_migration::prelude::*;

use crate::m20260727_180611_create_grid_worlds::GridWorlds;

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260806_120000_unique_grid_world_name_version"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    /// `(name, version)` identifies a grid snapshot.
    ///
    /// A grid row is immutable once a plan references it; editing one then forks a new
    /// row at `version + 1`. This index is what makes that fork safe: two writers
    /// forking the same snapshot both compute the same next version, and the second
    /// insert loses here rather than quietly producing two different grids that both
    /// claim to be v4.
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .create_index(
                Index::create()
                    .if_not_exists()
                    .name("idx-grid_worlds-name-version")
                    .table(GridWorlds::Table)
                    .col(GridWorlds::Name)
                    .col(GridWorlds::Version)
                    .unique()
                    .to_owned(),
            )
            .await
    }

    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .drop_index(
                Index::drop()
                    .name("idx-grid_worlds-name-version")
                    .table(GridWorlds::Table)
                    .to_owned(),
            )
            .await
    }
}