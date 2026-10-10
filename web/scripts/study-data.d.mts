export const STUDY_FILES: readonly ['demo-data.json', 'dynamic-data.json', 'predictive-data.json'];
export const MAX_ARCHIVE_PART_BYTES: number;
export interface StudyArchiveOptions { publicDir?: string; archiveDir?: string; partBytes?: number }
export interface StudyArchiveEntry {
  name: string; bytes: number; archiveBytes: number; sha256: string;
  parts: { archive: string; bytes: number; sha256: string }[];
}
export function packStudyData(options?: StudyArchiveOptions): Promise<StudyArchiveEntry[]>;
export function restoreStudyData(options?: StudyArchiveOptions): Promise<string[]>;
