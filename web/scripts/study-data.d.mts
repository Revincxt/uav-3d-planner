export const STUDY_FILES: readonly ['demo-data.json', 'dynamic-data.json', 'predictive-data.json'];
export interface StudyArchiveOptions { publicDir?: string; archiveDir?: string }
export interface StudyArchiveEntry {
  name: string; archive: string; bytes: number; archiveBytes: number; sha256: string;
}
export function packStudyData(options?: StudyArchiveOptions): Promise<StudyArchiveEntry[]>;
export function restoreStudyData(options?: StudyArchiveOptions): Promise<string[]>;
