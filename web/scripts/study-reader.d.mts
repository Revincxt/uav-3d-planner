export function parseStudyChunks(chunks: Iterable<string>): any;
export function readStudyData(file: string | URL): Promise<any>;
export function readStudyDataWithHash(file: string | URL): Promise<{ value: any; sha256: string }>;
